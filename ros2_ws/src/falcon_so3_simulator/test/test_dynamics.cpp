#include <limits>
#include <gtest/gtest.h>
#include "quadrotor_simulator/control.h"
using QuadrotorSimulator::Quadrotor;
using namespace falcon_so3_simulator;

TEST(QuadrotorDynamicsTest, EquilibriumHoverAndMotorLagAreIntegrated) {
  Quadrotor quad; quad.setMinRPM(0);
  auto state=quad.getState(); state.x={0,0,2};
  const double hover=std::sqrt(quad.getMass()*quad.getGravity()/(4*quad.getPropellerThrustCoefficient()));
  state.motor_rpm.setConstant(hover); quad.setState(state); quad.setInput(hover,hover,hover,hover);
  for(int i=0;i<2000;++i) quad.step(0.001);
  EXPECT_LT((quad.getState().x-Eigen::Vector3d(0,0,2)).norm(),1e-9);
  EXPECT_LT(quad.getState().v.norm(),1e-9);
  quad.setInput(hover*1.1,hover*1.1,hover*1.1,hover*1.1); quad.step(0.001);
  EXPECT_GT(quad.getState().motor_rpm[0],hover); EXPECT_LT(quad.getState().motor_rpm[0],hover*1.1);
  EXPECT_THROW(quad.step(0),std::invalid_argument);
  EXPECT_THROW(quad.setInput(std::numeric_limits<double>::quiet_NaN(),0,0,0),std::invalid_argument);
}

TEST(QuadrotorDynamicsTest, TracksThenBrakesWithFiniteRotorAndAttitudeResponse) {
  Quadrotor quad; quad.setMinRPM(0);
  auto state=quad.getState(); state.x={0,0,2};
  const double hover=std::sqrt(quad.getMass()*quad.getGravity()/(4*quad.getPropellerThrustCoefficient()));
  state.motor_rpm.setConstant(hover); quad.setState(state);
  Reference reference; reference.position=state.x;
  double max_error=0,max_speed=0,max_acceleration=0;
  for(int i=0;i<9000;++i) {
    const double t=i*0.001;
    if(t<2) { reference.position.x()=0.5*t*t; reference.velocity.x()=t; reference.acceleration.x()=1; }
    else if(t<3) { const double s=t-2; reference.position.x()=2+2*s-s*s; reference.velocity.x()=2-2*s; reference.acceleration.x()=-2; }
    else { reference.position.x()=3; reference.velocity.setZero(); reference.acceleration.setZero(); }
    const auto command=calculate(quad,reference,{1.5,1.5,1},{0.13,0.13,0.1});
    quad.setInput(command.rpm[0],command.rpm[1],command.rpm[2],command.rpm[3]); quad.step(0.001);
    max_error=std::max(max_error,(reference.position-quad.getState().x).norm());
    max_speed=std::max(max_speed,quad.getState().v.norm());
    max_acceleration=std::max(max_acceleration,quad.getAcc().norm());
  }
  EXPECT_LT(max_error,0.25); EXPECT_LT((quad.getState().x-reference.position).norm(),0.02);
  EXPECT_LT(quad.getState().v.norm(),0.02);
  EXPECT_GT(max_speed,1.5); EXPECT_GT(max_acceleration,1.5);
  EXPECT_LT((quad.getState().R.transpose()*quad.getState().R-Eigen::Matrix3d::Identity()).norm(),1e-10);
}
