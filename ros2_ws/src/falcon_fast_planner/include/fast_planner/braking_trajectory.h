#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Eigen>

namespace fast_planner {
struct TranslationState {
  Eigen::Vector3d position, velocity, acceleration, jerk;
};

// Constant-magnitude deceleration: C1 position/velocity, finite acceleration.
// Acceleration switches at entry/exit; jerk and attitude actuator limits are
// deliberately not claimed by this translational kinematic model.
class BrakingTrajectory {
public:
  BrakingTrajectory() = default;
  BrakingTrajectory(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                    double deceleration) : position_(position), velocity_(velocity) {
    if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(velocity.norm()) ||
        !std::isfinite(deceleration) || deceleration<=0)
      throw std::invalid_argument("Invalid braking state or deceleration");
    duration_ = velocity.norm()/deceleration;
  }
  double duration() const { return duration_; }
  TranslationState sample(double time) const {
    if (!std::isfinite(time)) throw std::invalid_argument("Nonfinite braking sample time");
    const double t=std::clamp(time,0.0,duration_);
    TranslationState state{position_,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
    if (duration_>0) {
      state.position += velocity_*(t-0.5*t*t/duration_);
      if (time<duration_) {
        state.velocity = velocity_*(1-t/duration_);
        state.acceleration = -velocity_/duration_;
      }
    }
    return state;
  }
private:
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()}, velocity_{Eigen::Vector3d::Zero()};
  double duration_{0};
};

// Exact segment/AABB distance: break at slab crossings; on each interval the
// squared distance is a quadratic with a clamped analytic minimum.
inline double segmentBoxSquaredDistance(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                                        const Eigen::Vector3d &low, const Eigen::Vector3d &high) {
  const Eigen::Vector3d delta=end-start;
  std::array<double,8> cuts{0,1};
  size_t count=2;
  for(int axis=0;axis<3;++axis) if (std::abs(delta[axis])>1e-12)
    for(double face : {low[axis],high[axis]}) {
      const double t=(face-start[axis])/delta[axis];
      if(t>0 && t<1) cuts[count++]=t;
    }
  std::sort(cuts.begin(),cuts.begin()+count);
  double best=std::numeric_limits<double>::infinity();
  for(size_t i=1;i<count;++i) {
    const double mid=0.5*(cuts[i-1]+cuts[i]);
    double quadratic=0,linear=0;
    for(int axis=0;axis<3;++axis) {
      const double coordinate=start[axis]+mid*delta[axis];
      if(coordinate>=low[axis] && coordinate<=high[axis]) continue;
      const double face=coordinate<low[axis] ? low[axis] : high[axis];
      quadratic+=delta[axis]*delta[axis];
      linear+=delta[axis]*(start[axis]-face);
    }
    const double t=quadratic>0 ? std::clamp(-linear/quadratic,cuts[i-1],cuts[i]) : mid;
    const Eigen::Vector3d point=start+t*delta;
    const Eigen::Vector3d gap=(low-point).cwiseMax(point-high).cwiseMax(0.0);
    best=std::min(best,gap.squaredNorm());
  }
  return best;
}

inline bool segmentBoxInterval(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                               const Eigen::Vector3d &low, const Eigen::Vector3d &high,
                               double &entry, double &exit) {
  entry=0; exit=1;
  const Eigen::Vector3d delta=end-start;
  for(int axis=0;axis<3;++axis) {
    if(std::abs(delta[axis])<1e-12) {
      if(start[axis]<low[axis] || start[axis]>high[axis]) return false;
    } else {
      double a=(low[axis]-start[axis])/delta[axis], b=(high[axis]-start[axis])/delta[axis];
      if(a>b) std::swap(a,b);
      entry=std::max(entry,a); exit=std::min(exit,b);
      if(entry>exit) return false;
    }
  }
  return true;
}
} // namespace fast_planner
