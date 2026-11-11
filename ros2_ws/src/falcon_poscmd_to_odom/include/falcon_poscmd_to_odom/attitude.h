#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falcon_poscmd_to_odom {

// Differentiate the same flatness attitude used by the lightweight simulator.
// This is a kinematic command mapping, not an integrated SO3 dynamics model.
inline bool attitudeAndBodyRate(const Eigen::Vector3d &acceleration,
                                const Eigen::Vector3d &jerk, double yaw, double yaw_rate,
                                Eigen::Quaterniond &orientation, Eigen::Vector3d &body_rate) {
  if (!acceleration.allFinite() || !jerk.allFinite() ||
      !std::isfinite(yaw) || !std::isfinite(yaw_rate)) return false;
  const Eigen::Vector3d alpha = acceleration+9.8*Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0);
  const Eigen::Vector3d lateral(-heading.y(),heading.x(),0);
  const Eigen::Vector3d raw_x = lateral.cross(alpha);
  if (raw_x.norm()<1e-9 || alpha.norm()<1e-9) return false;
  const Eigen::Vector3d x = raw_x.normalized();
  const Eigen::Vector3d raw_dx = (-yaw_rate*heading).cross(alpha)+lateral.cross(jerk);
  const Eigen::Vector3d dx = (raw_dx-x*x.dot(raw_dx))/raw_x.norm();
  const Eigen::Vector3d raw_y = alpha.cross(x);
  const Eigen::Vector3d y = raw_y.normalized();
  const Eigen::Vector3d raw_dy = jerk.cross(x)+alpha.cross(dx);
  const Eigen::Vector3d dy = (raw_dy-y*y.dot(raw_dy))/raw_y.norm();
  const Eigen::Vector3d z = x.cross(y);
  const Eigen::Vector3d dz = dx.cross(y)+x.cross(dy);
  Eigen::Matrix3d rotation;
  rotation.col(0)=x; rotation.col(1)=y; rotation.col(2)=z;
  orientation = Eigen::Quaterniond(rotation);
  const Eigen::Vector3d world_rate = 0.5*(x.cross(dx)+y.cross(dy)+z.cross(dz));
  body_rate = rotation.transpose()*world_rate;
  return orientation.coeffs().allFinite() && body_rate.allFinite();
}
} // namespace falcon_poscmd_to_odom
