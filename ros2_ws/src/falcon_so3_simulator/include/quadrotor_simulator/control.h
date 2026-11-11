#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <Eigen/Geometry>
#include "quadrotor_simulator/Quadrotor.h"

namespace falcon_so3_simulator {
struct Reference {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()},velocity{Eigen::Vector3d::Zero()},acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d kx{5.7,5.7,6.2},kv{3.4,3.4,4.0};
  double yaw{0};
  double maximum_control_acceleration{std::numeric_limits<double>::infinity()};
};
struct Control {
  Eigen::Vector3d force;
  Eigen::Quaterniond orientation;
  Eigen::Array4d rpm;
};
// Native double-precision form of upstream SO3 force/attitude feedback and
// plus-configuration rotor allocation. No ideal pose/velocity assignment.
inline Control calculate(const QuadrotorSimulator::Quadrotor &quad,const Reference &reference,
                         const Eigen::Vector3d &kr,const Eigen::Vector3d &kom) {
  const auto &state=quad.getState();
  Control result;
  result.force=reference.kx.asDiagonal()*(reference.position-state.x)+
      reference.kv.asDiagonal()*(reference.velocity-state.v)+
      quad.getMass()*(reference.acceleration+quad.getGravity()*Eigen::Vector3d::UnitZ());
  if(std::isnan(reference.maximum_control_acceleration) || reference.maximum_control_acceleration<=0)
    throw std::invalid_argument("Invalid control acceleration budget");
  // Bound the total feedback+feedforward request, not merely its reference
  // acceleration. This constrains actuator requests without clipping plant state.
  Eigen::Vector3d net_acceleration=result.force/quad.getMass()-quad.getGravity()*Eigen::Vector3d::UnitZ();
  if(net_acceleration.norm()>reference.maximum_control_acceleration)
    net_acceleration*=reference.maximum_control_acceleration/net_acceleration.norm();
  result.force=quad.getMass()*(net_acceleration+quad.getGravity()*Eigen::Vector3d::UnitZ());
  // Use the documented 45-degree tilt limit; upstream's comment said 45 but
  // its theta=pi/2 permitted horizontal thrust and singular attitude requests.
  result.force.z()=std::max(result.force.z(),0.1*quad.getMass()*quad.getGravity());
  const double horizontal=result.force.head<2>().norm();
  if(horizontal>result.force.z()) result.force.head<2>()*=result.force.z()/horizontal;
  const Eigen::Vector3d z=result.force.normalized();
  const Eigen::Vector3d heading(std::cos(reference.yaw),std::sin(reference.yaw),0);
  const Eigen::Vector3d y=z.cross(heading).normalized(),x=y.cross(z);
  Eigen::Matrix3d desired;
  desired.col(0)=x; desired.col(1)=y; desired.col(2)=z;
  result.orientation=Eigen::Quaterniond(desired);
  const Eigen::Matrix3d skew=0.5*(desired.transpose()*state.R-state.R.transpose()*desired);
  const Eigen::Vector3d error(skew(2,1),skew(0,2),skew(1,0));
  const Eigen::Vector3d moment=-(kr.asDiagonal()*error)-(kom.asDiagonal()*state.omega)+
      state.omega.cross(quad.getInertia()*state.omega);
  const double psi=0.5*(3-(desired.transpose()*state.R).trace());
  const double force=psi<1 ? std::max(0.0,result.force.dot(state.R.col(2))) : 0;
  const double kf=quad.getPropellerThrustCoefficient(),km=quad.getPropellerMomentCoefficient(),d=quad.getArmLength();
  const double lower=std::pow(quad.getMinRPM(),2),upper=std::pow(quad.getMaxRPM(),2);
  const double collective=std::clamp(force/(4*kf),lower,upper);
  Eigen::Array4d differential;
  differential << -moment.y()/(2*d*kf)+moment.z()/(4*km),
                   moment.y()/(2*d*kf)+moment.z()/(4*km),
                   moment.x()/(2*d*kf)-moment.z()/(4*km),
                  -moment.x()/(2*d*kf)-moment.z()/(4*km);
  // Preserve collective thrust under torque saturation. Clipping each motor
  // independently increases the total thrust when a negative request hits zero.
  double scale=1;
  // Retain half the collective on each rotor. A full zero-to-double yaw mix
  // loses collective during motor lag even when its steady-state sum is exact.
  const double mix_lower=std::max(lower,0.5*collective);
  const double mix_upper=std::min(upper,1.5*collective);
  for(int i=0;i<4;++i) {
    if(differential[i]>0) scale=std::min(scale,(mix_upper-collective)/differential[i]);
    if(differential[i]<0) scale=std::min(scale,(mix_lower-collective)/differential[i]);
  }
  result.rpm=(collective+scale*differential).max(lower).min(upper).sqrt();
  if(!result.force.allFinite() || !result.orientation.coeffs().allFinite() || !result.rpm.isFinite().all())
    throw std::runtime_error("Nonfinite SO3 control output");
  return result;
}
} // namespace falcon_so3_simulator
