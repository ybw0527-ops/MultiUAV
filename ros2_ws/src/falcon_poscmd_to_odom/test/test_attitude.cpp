#include <limits>
#include <gtest/gtest.h>
#include "falcon_poscmd_to_odom/attitude.h"

using falcon_poscmd_to_odom::attitudeAndBodyRate;

TEST(CommandAttitudeTest, ReportsYawRateAtHover) {
  Eigen::Quaterniond q;
  Eigen::Vector3d rate;
  ASSERT_TRUE(attitudeAndBodyRate(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),1.2,0.7,q,rate));
  EXPECT_TRUE(rate.isApprox(Eigen::Vector3d(0,0,0.7),1e-12));
}

TEST(CommandAttitudeTest, BodyRateMatchesIndependentAttitudeDifference) {
  const Eigen::Vector3d acceleration(0.6,-0.4,0.2), jerk(0.5,0.2,-0.1);
  constexpr double yaw=0.9, yaw_rate=-0.8, dt=1e-6;
  Eigen::Quaterniond q, before, after;
  Eigen::Vector3d rate, unused;
  ASSERT_TRUE(attitudeAndBodyRate(acceleration,jerk,yaw,yaw_rate,q,rate));
  ASSERT_TRUE(attitudeAndBodyRate(acceleration-dt*jerk,jerk,yaw-dt*yaw_rate,yaw_rate,before,unused));
  ASSERT_TRUE(attitudeAndBodyRate(acceleration+dt*jerk,jerk,yaw+dt*yaw_rate,yaw_rate,after,unused));
  const Eigen::Matrix3d skew = q.toRotationMatrix().transpose() *
      (after.toRotationMatrix()-before.toRotationMatrix())/(2*dt);
  const Eigen::Vector3d numeric(skew(2,1),skew(0,2),skew(1,0));
  EXPECT_LT((numeric-rate).norm(),1e-8);
  EXPECT_NEAR(q.norm(),1,1e-12);
}

TEST(CommandAttitudeTest, RejectsSingularAndNonfiniteInputs) {
  Eigen::Quaterniond q;
  Eigen::Vector3d rate;
  EXPECT_FALSE(attitudeAndBodyRate({0,0,-9.8},Eigen::Vector3d::Zero(),0,0,q,rate));
  EXPECT_FALSE(attitudeAndBodyRate(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
                                  std::numeric_limits<double>::quiet_NaN(),0,q,rate));
}
