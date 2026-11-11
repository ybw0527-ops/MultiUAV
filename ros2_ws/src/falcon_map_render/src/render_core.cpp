#include "falcon_map_render/render_core.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

namespace falcon_map_render {
void Camera::validate() const {
  if (width <= 0 || height <= 0 || width > 4096 || height > 4096 ||
      !std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
      !std::isfinite(cx) || !std::isfinite(cy) || cx < 0 || cy < 0 || cx >= width || cy >= height ||
      !std::isfinite(near) || !std::isfinite(far) || near <= 0 || far <= near || far > 60 ||
      !std::isfinite(point_radius) || point_radius < 0)
    throw std::invalid_argument("Invalid renderer camera configuration");
}
bool validTransform(const Eigen::Isometry3d &t) {
  return t.matrix().allFinite() &&
         (t.linear().transpose() * t.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) &&
         std::abs(t.linear().determinant() - 1.0) < 1e-5 &&
         t.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-5);
}
size_t Scene::primitiveCount() const { return mesh() ? triangles_.size() : points_.size(); }
void Scene::setTriangles(std::vector<Triangle> triangles) {
  if (triangles.empty()) throw std::invalid_argument("Mesh is empty");
  for (const auto &t : triangles)
    if (!t.a.allFinite() || !t.b.allFinite() || !t.c.allFinite())
      throw std::invalid_argument("Mesh has nonfinite vertices");
  triangles_ = std::move(triangles);
  points_.clear();
  order_.resize(triangles_.size());
  std::iota(order_.begin(), order_.end(), 0);
  bvh_.clear();
  build(0, triangles_.size());
}
void Scene::setPoints(std::vector<Eigen::Vector3d> points) {
  points.erase(std::remove_if(points.begin(), points.end(), [](const auto &p){return !p.allFinite();}), points.end());
  if (points.empty()) throw std::invalid_argument("Point cloud has no finite points");
  points_ = std::move(points);
  triangles_.clear(); bvh_.clear(); order_.clear();
}
void Scene::load(const std::string &filename, double scale, const Eigen::Isometry3d &map_from_world) {
  if (!std::isfinite(scale) || scale <= 0 || !validTransform(map_from_world))
    throw std::invalid_argument("Invalid map scale or transform");
  const auto world_from_map = map_from_world.inverse();
  auto extension = std::filesystem::path(filename).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c){return std::tolower(c);});
  if (extension == ".pcd" || extension == ".ply") {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const int status = extension == ".pcd" ? pcl::io::loadPCDFile(filename, cloud) : pcl::io::loadPLYFile(filename, cloud);
    if (status < 0) throw std::runtime_error("Cannot load point cloud: " + filename);
    std::vector<Eigen::Vector3d> points;
    points.reserve(cloud.size());
    for (const auto &p : cloud) points.push_back(world_from_map * (Eigen::Vector3d(p.x,p.y,p.z) / scale));
    setPoints(std::move(points));
  } else if (extension == ".stl" || extension == ".obj") {
    Assimp::Importer importer;
    const auto *scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_PreTransformVertices);
    if (!scene || !scene->HasMeshes()) throw std::runtime_error("Cannot load mesh: " + filename);
    std::vector<Triangle> triangles;
    for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
      const auto *m = scene->mMeshes[i];
      auto vertex = [&](unsigned int idx) -> Eigen::Vector3d {
        const auto &p = m->mVertices[idx];
        return world_from_map * (Eigen::Vector3d(p.x,p.y,p.z) / scale);
      };
      for (unsigned int j = 0; j < m->mNumFaces; ++j) {
        const auto &f = m->mFaces[j];
        if (f.mNumIndices == 3)
          triangles.push_back({vertex(f.mIndices[0]),vertex(f.mIndices[1]),vertex(f.mIndices[2])});
      }
    }
    setTriangles(std::move(triangles));
  } else throw std::invalid_argument("Unsupported map extension: " + extension);
}
int Scene::build(size_t begin, size_t end) {
  const int index = static_cast<int>(bvh_.size());
  BvhNode node;
  node.lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  node.hi = -node.lo;
  node.begin = begin; node.end = end;
  for (size_t i = begin; i < end; ++i) {
    const auto &t = triangles_[order_[i]];
    node.lo = node.lo.cwiseMin(t.a).cwiseMin(t.b).cwiseMin(t.c);
    node.hi = node.hi.cwiseMax(t.a).cwiseMax(t.b).cwiseMax(t.c);
  }
  bvh_.push_back(node);
  if (end - begin > 8) {
    Eigen::Index axis;
    (node.hi - node.lo).maxCoeff(&axis);
    const size_t mid = begin + (end - begin) / 2;
    std::nth_element(order_.begin()+begin, order_.begin()+mid, order_.begin()+end,
      [&](size_t a, size_t b) {
        const auto &ta=triangles_[a]; const auto &tb=triangles_[b];
        return ta.a[axis]+ta.b[axis]+ta.c[axis] < tb.a[axis]+tb.b[axis]+tb.c[axis];
      });
    const int left = build(begin, mid), right = build(mid, end);
    bvh_[index].left = left; bvh_[index].right = right;
  }
  return index;
}
double Scene::hit(int index, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
                  double near, double far) const {
  const auto &node = bvh_[index];
  double lo = near, hi = far;
  for (int a=0; a<3; ++a) {
    if (std::abs(direction[a]) < 1e-12) {
      if (origin[a] < node.lo[a] || origin[a] > node.hi[a]) return far;
    } else {
      double first=(node.lo[a]-origin[a])/direction[a], last=(node.hi[a]-origin[a])/direction[a];
      if (first > last) std::swap(first,last);
      lo=std::max(lo,first); hi=std::min(hi,last);
      if (lo > hi) return far;
    }
  }
  if (node.left >= 0) {
    far = hit(node.left,origin,direction,near,far);
    return hit(node.right,origin,direction,near,far);
  }
  for (size_t i=node.begin; i<node.end; ++i) {
    const auto &t=triangles_[order_[i]];
    const Eigen::Vector3d e1=t.b-t.a, e2=t.c-t.a;
    const Eigen::Vector3d p=direction.cross(e2);
    const double determinant=e1.dot(p);
    if (std::abs(determinant)<1e-12) continue;
    const Eigen::Vector3d offset=origin-t.a;
    const double u=offset.dot(p)/determinant;
    if (u<0 || u>1) continue;
    const Eigen::Vector3d q=offset.cross(e1);
    const double v=direction.dot(q)/determinant;
    if (v<0 || u+v>1) continue;
    const double depth=e2.dot(q)/determinant;
    if (depth>=near && depth<far) far=depth;
  }
  return far;
}
std::vector<float> Scene::render(const Camera &c, const Eigen::Isometry3d &pose) const {
  c.validate();
  if (!validTransform(pose) || primitiveCount()==0) throw std::invalid_argument("Invalid pose or empty scene");
  std::vector<float> depth(static_cast<size_t>(c.width)*c.height, static_cast<float>(c.far+1));
  if (mesh()) {
    for (int v=0; v<c.height; ++v) for (int u=0; u<c.width; ++u) {
      // Do not normalize: ray parameter is optical z depth, not Euclidean range.
      const Eigen::Vector3d direction = pose.linear()*Eigen::Vector3d((u-c.cx)/c.fx,(v-c.cy)/c.fy,1);
      const double d = hit(0, pose.translation(), direction, c.near, c.far+1e-6);
      if (d<=c.far) depth[static_cast<size_t>(v)*c.width+u]=static_cast<float>(d);
    }
  } else {
    const Eigen::Isometry3d camera_from_world=pose.inverse();
    for (const auto &point : points_) {
      const Eigen::Vector3d p=camera_from_world*point;
      if (p.z()<c.near || p.z()>c.far) continue;
      const double x=c.fx*p.x()/p.z()+c.cx, y=c.fy*p.y()/p.z()+c.cy;
      const double rx=std::min<double>(c.width, std::ceil(c.fx*c.point_radius/p.z()));
      const double ry=std::min<double>(c.height, std::ceil(c.fy*c.point_radius/p.z()));
      if (x+rx<0 || y+ry<0 || x-rx>c.width-1 || y-ry>c.height-1) continue;
      const int x0=static_cast<int>(std::max(0.0, std::floor(x-rx)));
      const int x1=static_cast<int>(std::min<double>(c.width-1, std::ceil(x+rx)));
      const int y0=static_cast<int>(std::max(0.0, std::floor(y-ry)));
      const int y1=static_cast<int>(std::min<double>(c.height-1, std::ceil(y+ry)));
      for (int v=y0; v<=y1; ++v) for (int u=x0; u<=x1; ++u)
        depth[static_cast<size_t>(v)*c.width+u]=std::min(depth[static_cast<size_t>(v)*c.width+u],static_cast<float>(p.z()));
    }
  }
  return depth;
}
} // namespace falcon_map_render
