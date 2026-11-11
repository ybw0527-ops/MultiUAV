#include <limits>
#include <gtest/gtest.h>
#include "quadrotor_simulator/control.h"
#include "quadrotor_simulator/execution_timeline.h"
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

TEST(ExecutionTimelineTest, ClockCatchupMatchesContinuousInputReplay) {
  Quadrotor normal,catchup;
  auto state=normal.getState(); state.x={0,0,2};
  const double hover=std::sqrt(normal.getMass()*normal.getGravity()/(4*normal.getPropellerThrustCoefficient()));
  state.motor_rpm.setConstant(hover); normal.setMinRPM(0); catchup.setMinRPM(0);
  normal.setState(state); catchup.setState(state);
  Reference initial; initial.position=state.x;
  ReferenceTimeline dense(initial),batch(initial);
  std::vector<Reference> inputs;
  for(int i=0;i<100;++i) {
    Reference input=initial;
    const double t=i*0.01;
    input.position.x()=0.5*t*t; input.velocity.x()=t; input.acceleration.x()=1;
    inputs.push_back(input); ASSERT_TRUE(batch.push(t,input));
  }
  for(int i=0;i<100;++i) {
    ASSERT_TRUE(dense.push(i*0.01,inputs[i]));
    advanceExecution(normal,dense,i*0.01,(i+1)*0.01,0.001,2,{1.5,1.5,1},{0.13,0.13,0.1});
  }
  size_t steps=0;
  advanceExecution(catchup,batch,0,1,0.001,2,{1.5,1.5,1},{0.13,0.13,0.1},[&](double){ ++steps; });
  EXPECT_GE(steps,1000U);
  EXPECT_LT((normal.getState().x-catchup.getState().x).norm(),1e-9);
  EXPECT_LT((normal.getState().v-catchup.getState().v).norm(),1e-9);
  EXPECT_LT((normal.getState().R-catchup.getState().R).norm(),1e-9);
  EXPECT_THROW(advanceExecution(catchup,batch,1,0.9,0.001,2,{1,1,1},{1,1,1}),std::invalid_argument);
  EXPECT_THROW(advanceExecution(catchup,batch,1,4,0.001,2,{1,1,1},{1,1,1}),std::invalid_argument);
}

TEST(ExecutionTimelineTest, FutureCommandsAndWatchdogUseTheirOwnEventTimes) {
  Reference initial; initial.position={0,0,2};
  Reference command=initial; command.velocity={1,0,0}; command.acceleration={0.5,0,0};
  ReferenceTimeline timeline(initial);
  ASSERT_TRUE(timeline.push(1.0,command));
  EXPECT_DOUBLE_EQ(timeline.at(0.8).velocity.norm(),0);
  EXPECT_DOUBLE_EQ(timeline.at(1.0).velocity.x(),1);
  EXPECT_DOUBLE_EQ(timeline.at(1.499).acceleration.x(),0.5);
  EXPECT_NEAR(timeline.at(1.499).velocity.x(),1.2495,1e-10);
  EXPECT_DOUBLE_EQ(timeline.at(1.5).velocity.x(),1.25);
  EXPECT_DOUBLE_EQ(timeline.at(1.5).acceleration.x(),-1.2);
  EXPECT_NEAR(timeline.at(3).velocity.norm(),0,1e-12);
  EXPECT_DOUBLE_EQ(timeline.at(3).acceleration.norm(),0);
  EXPECT_FALSE(timeline.push(0.9,command));
}

TEST(ExecutionTimelineTest, MissingInputsPredictThenBrakeWithoutMovingTheHoldBackward) {
  Reference input; input.position={0,0,2}; input.velocity={1.5,0,0};
  input.maximum_control_acceleration=1.5;
  ReferenceTimeline timeline(input); ASSERT_TRUE(timeline.push(0,input));
  EXPECT_NEAR(timeline.at(0.5-1e-8).position.x(),0.75,2e-8);
  EXPECT_NEAR(timeline.at(0.5).position.x(),0.75,1e-12);
  EXPECT_NEAR(timeline.at(0.8).position.x(),1.146,1e-12);
  EXPECT_NEAR(timeline.at(0.8).velocity.x(),1.14,1e-12);
  EXPECT_NEAR(timeline.at(2).position.x(),1.6875,1e-12);
  EXPECT_NEAR(timeline.at(2).velocity.norm(),0,1e-12);
  Reference accelerating=input; accelerating.velocity={1.6,0,0}; accelerating.acceleration={1.2,0,0};
  ReferenceTimeline capped(accelerating); ASSERT_TRUE(capped.push(0,accelerating));
  const double sample_time=0.3,epsilon=1e-6;
  EXPECT_NEAR(capped.at(sample_time).velocity.norm(),1.7,1e-12);
  EXPECT_NEAR(capped.at(sample_time).acceleration.norm(),0,1e-12);
  EXPECT_LT(((capped.at(sample_time+epsilon).position-capped.at(sample_time-epsilon).position)/(2*epsilon)-
              capped.at(sample_time).velocity).norm(),1e-8);
  Quadrotor quad; quad.setMinRPM(0);
  auto state=quad.getState(); state.x=input.position; state.v=input.velocity;
  const double hover=std::sqrt(quad.getMass()*quad.getGravity()/(4*quad.getPropellerThrustCoefficient()));
  state.motor_rpm.setConstant(hover); quad.setState(state);
  ReferenceTimeline delayed(input); ASSERT_TRUE(delayed.push(0,input));
  double peak_acceleration=0;
  advanceExecution(quad,delayed,0,0.8,0.001,2,{1.5,1.5,1},{0.13,0.13,0.1},
      [&](double) { peak_acceleration=std::max(peak_acceleration,quad.getAcc().norm()); });
  EXPECT_LT(std::abs(quad.getState().x.x()-1.2),0.15);
  EXPECT_LT(peak_acceleration,2);
}

TEST(SO3ControlTest, FeedbackAndFeedforwardShareOneForceBudget) {
  Quadrotor quad; quad.setMinRPM(0);
  auto state=quad.getState(); state.x={0,0,2};
  const double hover=std::sqrt(quad.getMass()*quad.getGravity()/(4*quad.getPropellerThrustCoefficient()));
  state.motor_rpm.setConstant(hover); quad.setState(state);
  Reference reference; reference.position={1,0,2}; reference.maximum_control_acceleration=1.2;
  double peak_acceleration=0;
  for(int i=0;i<6000;++i) {
    const auto output=calculate(quad,reference,{1.5,1.5,1},{0.13,0.13,0.1});
    EXPECT_LE((output.force/quad.getMass()-quad.getGravity()*Eigen::Vector3d::UnitZ()).norm(),1.2+1e-10);
    quad.setInput(output.rpm[0],output.rpm[1],output.rpm[2],output.rpm[3]); quad.step(0.001);
    peak_acceleration=std::max(peak_acceleration,quad.getAcc().norm());
  }
  EXPECT_GT(peak_acceleration,0.5); EXPECT_LT(peak_acceleration,2);
  EXPECT_LT((quad.getState().x-reference.position).norm(),0.03);
}

TEST(SO3ControlTest, TorqueSaturationPreservesCollectiveThrust) {
  Quadrotor quad; quad.setMinRPM(0);
  auto state=quad.getState(); state.x={0,0,2};
  const double hover=std::sqrt(quad.getMass()*quad.getGravity()/(4*quad.getPropellerThrustCoefficient()));
  state.motor_rpm.setConstant(hover); quad.setState(state);
  Reference reference; reference.position=state.x; reference.yaw=1.5;
  reference.maximum_control_acceleration=1.5;
  double peak_acceleration=0;
  for(int i=0;i<6000;++i) {
    const auto output=calculate(quad,reference,{1.5,1.5,1},{0.13,0.13,0.1});
    const double actual_requested_thrust=quad.getPropellerThrustCoefficient()*output.rpm.square().sum();
    EXPECT_NEAR(actual_requested_thrust,output.force.dot(quad.getState().R.col(2)),1e-10);
    quad.setInput(output.rpm[0],output.rpm[1],output.rpm[2],output.rpm[3]); quad.step(0.001);
    peak_acceleration=std::max(peak_acceleration,quad.getAcc().norm());
  }
  EXPECT_LT(peak_acceleration,2);
  EXPECT_LT((quad.getState().x-reference.position).norm(),0.03);
  EXPECT_LT(std::abs(std::atan2(quad.getState().R(1,0),quad.getState().R(0,0))-reference.yaw),0.02);
}

TEST(SO3ControlTest, FullPresetSpeedBrakesThroughRotorDynamics) {
  for(const Eigen::Vector3d direction : {Eigen::Vector3d::UnitX().eval(),Eigen::Vector3d(1,1,0).normalized().eval()}) {
    Quadrotor quad; quad.setMinRPM(0);
    auto state=quad.getState(); state.x={0,0,2};
    const double hover=std::sqrt(quad.getMass()*quad.getGravity()/(4*quad.getPropellerThrustCoefficient()));
    state.motor_rpm.setConstant(hover); quad.setState(state);
    Reference reference; reference.maximum_control_acceleration=1.5;
    const double speed=1.7,acceleration=1.2,ramp=speed/acceleration,cruise=0.5;
    double peak_speed=0,peak_acceleration=0,peak_error=0;
    for(int i=0;i<9000;++i) {
      const double t=i*0.001;
      double p,v,a;
      if(t<ramp) { p=0.5*acceleration*t*t; v=acceleration*t; a=acceleration; }
      else if(t<ramp+cruise) { p=0.5*speed*ramp+speed*(t-ramp); v=speed; a=0; }
      else if(t<2*ramp+cruise) {
        const double elapsed=t-ramp-cruise;
        p=0.5*speed*ramp+speed*cruise+speed*elapsed-0.5*acceleration*elapsed*elapsed;
        v=speed-acceleration*elapsed; a=-acceleration;
      } else { p=speed*(ramp+cruise); v=0; a=0; }
      reference.position=state.x+p*direction; reference.velocity=v*direction; reference.acceleration=a*direction;
      const auto output=calculate(quad,reference,{1.5,1.5,1},{0.13,0.13,0.1});
      quad.setInput(output.rpm[0],output.rpm[1],output.rpm[2],output.rpm[3]); quad.step(0.001);
      peak_speed=std::max(peak_speed,quad.getState().v.norm());
      peak_acceleration=std::max(peak_acceleration,quad.getAcc().norm());
      peak_error=std::max(peak_error,(quad.getState().x-reference.position).norm());
    }
    EXPECT_GT(peak_speed,1.6); EXPECT_LT(peak_speed,2);
    EXPECT_LT(peak_acceleration,2); EXPECT_LT(peak_error,0.3);
    EXPECT_LT(quad.getState().v.norm(),0.001);
    EXPECT_LT((quad.getState().x-reference.position).norm(),0.001);
  }
}
