#pragma once
#include <deque>
#include <limits>
#include <functional>
#include "quadrotor_simulator/control.h"

namespace falcon_so3_simulator {
// Timestamped reference states. Between events, advance the received p/v/a
// consistently; after the watchdog, brake the predicted state to a fixed hold.
class ReferenceTimeline {
public:
  explicit ReferenceTimeline(const Reference &initial=Reference(),double speed_limit=1.7,
                             double braking_acceleration=1.2)
      : active_(initial),speed_limit_(speed_limit),braking_acceleration_(braking_acceleration) {
    if(!std::isfinite(speed_limit_) || speed_limit_<=0 ||
       !std::isfinite(braking_acceleration_) || braking_acceleration_<=0)
      throw std::invalid_argument("Invalid reference prediction limits");
  }
  bool push(double stamp,const Reference &reference) {
    if(!std::isfinite(stamp)) throw std::invalid_argument("Nonfinite command event time");
    if(stamp<last_received_) return false;
    if(pending_.size()>=4096) throw std::runtime_error("SO3 command timeline overflow");
    if(!pending_.empty() && stamp==pending_.back().stamp) pending_.back().reference=reference;
    else pending_.push_back({stamp,reference});
    last_received_=stamp;
    return true;
  }
  Reference at(double time) {
    if(!std::isfinite(time)) throw std::invalid_argument("Nonfinite reference time");
    while(!pending_.empty() && pending_.front().stamp<=time+1e-12) {
      active_=pending_.front().reference; active_stamp_=pending_.front().stamp;
      pending_.pop_front();
    }
    if(!std::isfinite(active_stamp_)) return active_;
    const double age=std::max(0.0,time-active_stamp_);
    if(age<0.5) return predict(age);
    auto result=predict(0.5);
    const double speed=result.velocity.norm(),duration=speed/braking_acceleration_;
    const Eigen::Vector3d deceleration=speed>1e-12 ?
        (-braking_acceleration_*result.velocity/speed).eval() : Eigen::Vector3d::Zero().eval();
    const double elapsed=std::min(age-0.5,duration);
    result.position+=elapsed*result.velocity+0.5*elapsed*elapsed*deceleration;
    result.velocity+=elapsed*deceleration;
    result.acceleration=age-0.5<duration ? deceleration : Eigen::Vector3d::Zero().eval();
    return result;
  }
  double boundaryAfter(double time) const {
    double result=pending_.empty() ? std::numeric_limits<double>::infinity() : pending_.front().stamp;
    const double expiry=active_stamp_+0.5;
    if(expiry>time+1e-12) result=std::min(result,expiry);
    return result;
  }
private:
  Reference predict(double elapsed) const {
    auto result=active_;
    // Constant acceleration prediction stops accelerating when the speed norm
    // reaches the configured reference limit, then continues at that velocity.
    double accelerated=elapsed;
    const double squared_acceleration=result.acceleration.squaredNorm();
    if(squared_acceleration>1e-18) {
      const double product=result.velocity.dot(result.acceleration);
      const double discriminant=product*product+squared_acceleration*
          std::max(0.0,speed_limit_*speed_limit_-result.velocity.squaredNorm());
      accelerated=std::min(elapsed,std::max(0.0,(-product+std::sqrt(discriminant))/squared_acceleration));
    }
    result.position+=accelerated*result.velocity+0.5*accelerated*accelerated*result.acceleration;
    result.velocity+=accelerated*result.acceleration;
    result.position+=(elapsed-accelerated)*result.velocity;
    if(accelerated<elapsed) result.acceleration.setZero();
    return result;
  }
  struct Event { double stamp; Reference reference; };
  std::deque<Event> pending_;
  Reference active_;
  double active_stamp_{-std::numeric_limits<double>::infinity()};
  double last_received_{-std::numeric_limits<double>::infinity()};
  double speed_limit_,braking_acceleration_;
};

inline void advanceExecution(QuadrotorSimulator::Quadrotor &quad,ReferenceTimeline &timeline,
                             double begin,double end,double maximum_step,double maximum_gap,
                             const Eigen::Vector3d &kr,const Eigen::Vector3d &kom,
                             const std::function<void(double)> &observe={}) {
  if(!std::isfinite(begin) || !std::isfinite(end) || end<begin || end-begin>maximum_gap ||
      !std::isfinite(maximum_step) || maximum_step<=0 || maximum_step>0.01 ||
      !std::isfinite(maximum_gap) || maximum_gap<=0)
    throw std::invalid_argument("Invalid SO3 execution time interval");
  double time=begin;
  while(time<end-1e-12) {
    const auto reference=timeline.at(time);
    const double next=std::min({end,time+maximum_step,timeline.boundaryAfter(time)});
    if(next<=time) throw std::runtime_error("Nonadvancing SO3 event timeline");
    const auto control=calculate(quad,reference,kr,kom);
    quad.setInput(control.rpm[0],control.rpm[1],control.rpm[2],control.rpm[3]);
    quad.step(next-time);
    time=next;
    if(observe) observe(time);
  }
}
} // namespace falcon_so3_simulator
