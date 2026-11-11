#ifndef FALCON_MAP_RENDER_CORE_H_
#define FALCON_MAP_RENDER_CORE_H_
#include <Eigen/Geometry>
#include <cstddef>
#include <string>
#include <vector>

namespace falcon_map_render {
struct Camera {
  int width{160}, height{120};
  double fx{80}, fy{80}, cx{80}, cy{60};
  double near{0.1}, far{5.0}, point_radius{0.1};
  void validate() const;
};
struct Triangle { Eigen::Vector3d a, b, c; };
class Scene {
public:
  void load(const std::string &filename, double scale = 1.0,
            const Eigen::Isometry3d &map_from_world = Eigen::Isometry3d::Identity());
  void setTriangles(std::vector<Triangle> triangles);
  void setPoints(std::vector<Eigen::Vector3d> points);
  // Axial optical-camera depth in meters. Missing returns far + 1, as a free ray.
  std::vector<float> render(const Camera &camera, const Eigen::Isometry3d &world_from_camera) const;
  size_t primitiveCount() const;
  bool mesh() const { return !triangles_.empty(); }
private:
  struct BvhNode {
    Eigen::Vector3d lo, hi;
    int left{-1}, right{-1};
    size_t begin{0}, end{0};
  };
  int build(size_t begin, size_t end);
  double hit(int node, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
             double near, double far) const;
  std::vector<Triangle> triangles_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<size_t> order_;
  std::vector<BvhNode> bvh_;
};
bool validTransform(const Eigen::Isometry3d &transform);
} // namespace falcon_map_render
#endif
