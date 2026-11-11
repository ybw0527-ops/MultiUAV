#include <gtest/gtest.h>
#include "falcon_map_render/render_core.h"
#include <filesystem>
#include <limits>
using namespace falcon_map_render;

TEST(RenderCore, NearestTriangleHasAxialDepthAndMovesWithCamera) {
  Scene scene;
  scene.setTriangles({{{-10,-10,2},{10,-10,2},{0,10,2}},
                      {{-10,-10,4},{10,-10,4},{0,10,4}}});
  Camera camera; camera.width=5; camera.height=5; camera.fx=2; camera.fy=2; camera.cx=2; camera.cy=2;
  auto pose=Eigen::Isometry3d::Identity();
  auto depth=scene.render(camera,pose);
  EXPECT_FLOAT_EQ(depth[12],2.0F);
  EXPECT_FLOAT_EQ(depth[13],2.0F); // Off-axis ray must not return Euclidean range.
  pose.translation().z()=0.5;
  EXPECT_FLOAT_EQ(scene.render(camera,pose)[12],1.5F);
  camera.far=1.0;
  EXPECT_FLOAT_EQ(scene.render(camera,pose)[12],2.0F); // Missing is far+1.
}

TEST(RenderCore, PointProjectionZBufferAndInvalidInputs) {
  Scene scene;
  scene.setPoints({{0,0,4},{0,0,2},{0,0,-1},{std::numeric_limits<double>::quiet_NaN(),0,1}});
  Camera camera; camera.width=5; camera.height=5; camera.fx=2; camera.fy=2; camera.cx=2; camera.cy=2; camera.point_radius=0;
  EXPECT_FLOAT_EQ(scene.render(camera,Eigen::Isometry3d::Identity())[12],2.0F);
  camera.fx=0;
  EXPECT_THROW(scene.render(camera,Eigen::Isometry3d::Identity()),std::invalid_argument);
  EXPECT_THROW(scene.load("bad.stl",0),std::invalid_argument);
  auto pose=Eigen::Isometry3d::Identity(); pose.linear()(0,0)=2;
  EXPECT_FALSE(validTransform(pose));
}

TEST(RenderCore, BvhFindsNearestAmongManyTriangles) {
  Scene scene;
  std::vector<Triangle> triangles;
  for(int i=1;i<=40;++i) {
    const double z=i*0.1+1;
    triangles.push_back({{-5,-5,z},{5,-5,z},{0,5,z}});
  }
  scene.setTriangles(triangles);
  Camera camera;
  EXPECT_NEAR(scene.render(camera,Eigen::Isometry3d::Identity())[60*160+80],1.1,1e-6);
}

TEST(RenderCore, LoadsAllSixUpstreamMaps) {
  for (const auto *name : {"classical_office.pcd","complex_office.stl","duplex_office.stl",
                            "octa_maze.stl","power_plant.pcd","darpa_tunnel.pcd"}) {
    SCOPED_TRACE(name);
    Scene scene;
    scene.load((std::filesystem::path(FALCON_MAP_RESOURCE_DIR)/name).string());
    EXPECT_GT(scene.primitiveCount(),0U);
  }
}
