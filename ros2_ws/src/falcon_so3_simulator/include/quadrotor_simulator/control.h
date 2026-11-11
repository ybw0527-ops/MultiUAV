#pragma once
#include <cmath>
#include <stdexcept>
#include <Eigen/Geometry>
#include "quadrotor_simulator/Quadrotor.h"

namespace falcon_so3_simulator {
struct Reference {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()},velocity{Eigen::Vector3d::Zero()},acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d kx{5.7,5.7,6.2},kv{3.4,3.4,4.0};
  double yaw{0};
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
  Eigen::Array4d squared;
  squared << force/(4*kf)-moment.y()/(2*d*kf)+moment.z()/(4*km),
             force/(4*kf)+moment.y()/(2*d*kf)+moment.z()/(4*km),
             force/(4*kf)+moment.x()/(2*d*kf)-moment.z()/(4*km),
             force/(4*kf)-moment.x()/(2*d*kf)-moment.z()/(4*km);
  result.rpm=squared.max(0.0).sqrt().min(quad.getMaxRPM()).max(quad.getMinRPM());
  if(!result.force.allFinite() || !result.orientation.coeffs().allFinite() || !result.rpm.isFinite().all())
    throw std::runtime_error("Nonfinite SO3 control output");
  return result;
}
} // namespace falcon_so3_simulator
