#pragma once
#include "fast_planner/braking_trajectory.h"

namespace fast_planner {
// Vertical then horizontal minimum-jerk segments, both with stationary ends.
class InitializationTrajectory {
public:
  InitializationTrajectory() = default;
  InitializationTrajectory(const Eigen::Vector3d &start, const Eigen::Vector3d &offset,
                           double velocity, double acceleration) : start_(start), offset_(offset) {
    if(!start.allFinite() || !offset.allFinite() || !std::isfinite(velocity) ||
       !std::isfinite(acceleration) || velocity<=0 || acceleration<=0)
      throw std::invalid_argument("Invalid initialization trajectory parameters");
    const auto time=[&](double length) {
      return std::max(1.875*length/velocity,std::sqrt(10/std::sqrt(3.0)*length/acceleration));
    };
    vertical_time_=time(std::abs(offset.z()));
    horizontal_time_=time(offset.head<2>().norm());
  }
  double duration() const { return vertical_time_+horizontal_time_; }
  TranslationState sample(double time) const {
    if(!std::isfinite(time)) throw std::invalid_argument("Nonfinite initialization time");
    const Eigen::Vector3d vertical(0,0,offset_.z()), horizontal(offset_.x(),offset_.y(),0);
    if(time>=duration()) return {start_+offset_,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
    const bool up=time<vertical_time_;
    const double interval=up ? vertical_time_ : horizontal_time_;
    if(interval<=0) return {start_+offset_,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
    const Eigen::Vector3d delta=up ? vertical : horizontal;
    const Eigen::Vector3d base=up ? start_ : (start_+vertical).eval();
    const double u=std::clamp((time-(up ? 0 : vertical_time_))/interval,0.0,1.0);
    const double s=u*u*u*(10-15*u+6*u*u);
    const double v=30*u*u*(1-u)*(1-u)/interval;
    const double a=60*u*(1-u)*(1-2*u)/(interval*interval);
    const double j=(60-360*u+360*u*u)/(interval*interval*interval);
    return {base+s*delta,v*delta,a*delta,j*delta};
  }
private:
  Eigen::Vector3d start_{Eigen::Vector3d::Zero()},offset_{Eigen::Vector3d::Zero()};
  double vertical_time_{0},horizontal_time_{0};
};
} // namespace fast_planner
