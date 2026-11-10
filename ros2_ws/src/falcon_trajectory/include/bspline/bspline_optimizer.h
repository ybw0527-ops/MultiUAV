#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <nlopt.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxel_mapping/map_server.h"

using namespace voxel_mapping;

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace fast_planner {

class BsplineOptimizer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<BsplineOptimizer> Ptr;
  typedef std::shared_ptr<const BsplineOptimizer> ConstPtr;

  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int START;
  static const int END;
  static const int GUIDE;
  static const int WAYPOINTS;
  static const int VIEWCONS;
  static const int MINTIME;
  static const int INFO;
  static const int YAWFEASIBILITY;

  static const int EXPLORATION_POSITION_PHASE;
  static const int EXPLORATION_YAW_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  void setParam(rclcpp::Node &node);
  void setMap(const MapServer::Ptr &map_server);
  void setCostFunction(const int &cost_function);
  void setBoundaryStates(const vector<Eigen::Vector3d> &start, const vector<Eigen::Vector3d> &end);
  void setTimeLowerBound(const double &lb);
  void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                    const vector<int> &waypt_idx); // N-2 constraints at most
  void setPhysicalLimits(const double &max_vel, const double &max_acc);

  int getBsplineDegree() { return bspline_degree_; }

  void optimize(Eigen::MatrixXd &points, double &dt, const int &cost_function);
  void optimize();

private:
  // Cost functions, q: control points, dt: knot span
  void calcSmoothnessCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                          vector<Eigen::Vector3d> &gradient_q, double &gt);
  void calcDistanceCost(const vector<Eigen::Vector3d> &q, double &cost,
                        vector<Eigen::Vector3d> &gradient_q);
  void calcFeasibilityCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                           vector<Eigen::Vector3d> &gradient_q, double &gt);
  void calcStartCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                     vector<Eigen::Vector3d> &gradient_q, double &gt);
  void calcEndCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                   vector<Eigen::Vector3d> &gradient_q, double &gt);
  void calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost,
                         vector<Eigen::Vector3d> &gradient_q);
  void calcTimeCost(const double &dt, double &cost, double &gt);

  // Wrapper of cost function
  void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);
  static double costFunction(const std::vector<double> &x, std::vector<double> &grad,
                             void *func_data);

  MapServer::Ptr map_server_;

  // Optimized variables
  Eigen::MatrixXd control_points_; // B-spline control points, N x dim
  double knot_span_;               // B-spline knot span

  // Input to solver
  int dim_{0}; // dimension of the B-spline
  vector<Eigen::Vector3d> start_state_, end_state_;
  vector<Eigen::Vector3d> waypoints_; // waypts constraints
  vector<int> waypt_idx_;
  int cost_function_{0};
  double time_lb_{-1.0};
  double start_time_{0.0}; // global time for moving obstacles

  // Parameters of optimization
  int order_{3};
  int bspline_degree_{3};
  double pos_smooth_{20.0}, pos_dist_{50.0}, pos_feasi_{1.0}, pos_start_{100.0};
  double pos_end_{50.0}, pos_time_{3.0};
  double yaw_smooth_{50.0}, yaw_start_{50.0}, yaw_end_{50.0}, yaw_waypt_{50.0};
  double safe_distance_{0.7};      // safe distance
  double max_vel_{1.0}, max_acc_{1.0};  // dynamic limits for pos
  int max_iteration_num_{1000};     // stopping criteria that can be used
  double max_iteration_time_{0.01}; // stopping criteria that can be used

  // Data of opt
  vector<Eigen::Vector3d> g_q_, g_smoothness_, g_distance_, g_feasibility_, g_start_, g_end_,
      g_waypoints_, g_time_;

  int variable_num_{0}; // optimization variables
  int point_num_{0};
  bool optimize_time_{false};
  std::vector<double> best_variable_;
  double min_cost_{std::numeric_limits<double>::max()};
  double pt_dist_{0.0};

  // debug
  std::vector<double> final_costs_;
  rclcpp::Node *node_{nullptr};
};
} // namespace fast_planner
#endif
