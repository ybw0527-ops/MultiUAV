#include <gtest/gtest.h>
#include "fast_planner/braking_trajectory.h"
#include "fast_planner/initialization_trajectory.h"

using namespace fast_planner;

TEST(BrakingTrajectoryTest, StopsTwoMetersPerSecondInOneMeterAndOneSecond) {
  BrakingTrajectory brake({0,0,1},{2,0,0},2);
  EXPECT_DOUBLE_EQ(brake.duration(),1);
  const auto moving=brake.sample(0.3), stopped=brake.sample(1);
  EXPECT_NEAR(moving.position.x(),0.51,1e-12);
  EXPECT_NEAR(moving.velocity.x(),1.4,1e-12);
  EXPECT_NEAR(moving.acceleration.norm(),2,1e-12);
  EXPECT_TRUE(stopped.position.isApprox(Eigen::Vector3d(1,0,1),1e-12));
  EXPECT_DOUBLE_EQ(stopped.velocity.norm(),0);
  EXPECT_DOUBLE_EQ(stopped.acceleration.norm(),0);
  EXPECT_TRUE(brake.sample(10).position.isApprox(stopped.position));
}

TEST(BrakingTrajectoryTest, DiagonalDecelerationUsesVectorNormAndConsistentPosition) {
  const Eigen::Vector3d velocity=Eigen::Vector3d(1,2,-1).normalized()*2;
  BrakingTrajectory brake({2,3,4},velocity,2);
  constexpr double dt=1e-4;
  auto previous=brake.sample(0);
  for(int i=1;i<=10000;++i) {
    const auto state=brake.sample(i*dt);
    EXPECT_LE((state.velocity-previous.velocity).norm()/dt,2+1e-9);
    EXPECT_LT(((state.position-previous.position)/dt-0.5*(state.velocity+previous.velocity)).norm(),1e-8);
    previous=state;
  }
  EXPECT_TRUE(previous.position.isApprox(Eigen::Vector3d(2,3,4)+0.5*velocity,1e-12));
  EXPECT_THROW(BrakingTrajectory({0,0,0},{1,0,0},0),std::invalid_argument);
}

TEST(BrakingGeometryTest, CapsuleDistanceIncludesInteriorsAndCornerMisses) {
  const Eigen::Vector3d low(0,0,0),high(1,1,1);
  EXPECT_DOUBLE_EQ(segmentBoxSquaredDistance({-2,0.5,0.5},{2,0.5,0.5},low,high),0);
  EXPECT_NEAR(segmentBoxSquaredDistance({-2,2,0.5},{2,2,0.5},low,high),1,1e-12);
  EXPECT_NEAR(segmentBoxSquaredDistance({2,2,2},{2,2,2},low,high),3,1e-12);
  // Compare the analytic result with an independent dense point/box oracle.
  for(int trial=0;trial<20;++trial) {
    const Eigen::Vector3d a(-2,0.1*trial-0.4,1.5),b(2,1.6-0.1*trial,-0.5);
    double numeric=100;
    for(int i=0;i<=10000;++i) {
      const Eigen::Vector3d p=a+(i/10000.0)*(b-a);
      const Eigen::Vector3d nearest=p.cwiseMax(low).cwiseMin(high);
      numeric=std::min(numeric,(p-nearest).squaredNorm());
    }
    const double exact=segmentBoxSquaredDistance(a,b,low,high);
    EXPECT_LE(exact,numeric+1e-12); EXPECT_NEAR(exact,numeric,1e-6);
  }
}

TEST(InitializationTrajectoryTest, SmoothBoundedVerticalThenHorizontalMotion) {
  InitializationTrajectory trajectory({2,3,0},{-0.6,0.8,1},2,2);
  double max_velocity=0,max_acceleration=0;
  auto previous=trajectory.sample(0);
  EXPECT_EQ(previous.velocity.norm(),0); EXPECT_EQ(previous.acceleration.norm(),0);
  for(double t=0.001;t<trajectory.duration();t+=0.001) {
    const auto state=trajectory.sample(t);
    max_velocity=std::max(max_velocity,state.velocity.norm());
    max_acceleration=std::max(max_acceleration,state.acceleration.norm());
    EXPECT_LE((state.velocity-previous.velocity).norm()/0.001,2+1e-5);
    previous=state;
  }
  EXPECT_LE(max_velocity,2+1e-8); EXPECT_LE(max_acceleration,2+1e-8);
  const auto end=trajectory.sample(trajectory.duration());
  EXPECT_TRUE(end.position.isApprox(Eigen::Vector3d(1.4,3.8,1)));
  EXPECT_EQ(end.velocity.norm(),0); EXPECT_EQ(end.acceleration.norm(),0);
}
