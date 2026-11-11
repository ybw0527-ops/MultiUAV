
#include "exploration_manager/exploration_fsm.h"
#include "exploration_manager/exploration_manager.h"
#include "exploration_manager/exploration_data.h"
#include "fast_planner/planner_manager.h"
#include "tic_toc.h"
#include "traj_utils/planning_visualization.h"

using Eigen::Vector4d;

namespace fast_planner {
namespace {
template<typename T> void readParameter(rclcpp::Node &node, const std::string &name, T &value, const T &fallback) {
 if (node.has_parameter(name)) node.get_parameter(name, value);
 else value = node.declare_parameter<T>(name, fallback);
}
}

void ExplorationFSM::init(rclcpp::Node &node) {
  node_ = &node;
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  readParameter(node, "exploration_manager.fsm.replan_thresh1", fp_->replan_thresh1_, 0.05);
  readParameter(node, "exploration_manager.fsm.replan_thresh2", fp_->replan_thresh2_, 0.2);
  readParameter(node, "exploration_manager.fsm.replan_thresh3", fp_->replan_thresh3_, 3.0);
  readParameter(node, "exploration_manager.fsm.replan_duration_fast", fp_->replan_duration_fast_, 0.05);
  readParameter(node, "exploration_manager.fsm.replan_duration_default", fp_->replan_duration_default_, 0.1);
  readParameter(node, "exploration_manager.fsm.replan_duration_slow", fp_->replan_duration_slow_, 0.2);
  fp_->replan_duration_ = fp_->replan_duration_default_;
  if (!std::isfinite(fp_->replan_duration_fast_) || !std::isfinite(fp_->replan_duration_default_) ||
      !std::isfinite(fp_->replan_duration_slow_) || fp_->replan_duration_fast_ <= 0.0 ||
      fp_->replan_duration_default_ < fp_->replan_duration_fast_ ||
      fp_->replan_duration_slow_ < fp_->replan_duration_default_ ||
      fp_->replan_thresh1_ < 0.0 || fp_->replan_thresh2_ < 0.0 || fp_->replan_thresh3_ <= 0.0)
    throw std::invalid_argument("Invalid FSM replan parameters");

  /* Initialize main modules */
  expl_manager_.reset(new ExplorationManager);
  expl_manager_->initialize(node);
  expl_manager_->ep_->replan_duration_ = fp_->replan_duration_;
  visualization_ = expl_manager_->visualization_;

  planner_manager_ = expl_manager_->planner_manager_;
  expl_manager_->visualization_ = visualization_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH", "RTB"};
  fd_->static_state_ = true;
  fd_->triggered_ = false;

  frontier_ready_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = node.create_wall_timer(std::chrono::milliseconds(10), [this]{ FSMCallback(); });
  safety_timer_ = node.create_wall_timer(std::chrono::milliseconds(50), [this]{ safetyCallback(); });
  frontier_timer_ = node.create_wall_timer(std::chrono::milliseconds(500), [this]{ frontierCallback(); });
  trigger_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){ triggerCallback(msg); });
  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>("/odom_world", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){ odometryCallback(msg); });
  replan_pub_ = node.create_publisher<std_msgs::msg::Int32>("/planning/replan", 10);
  bspline_pub_ = node.create_publisher<falcon_interfaces::msg::Bspline>("/planning/bspline", 10);
  uncertainty_pub_ = node.create_publisher<std_msgs::msg::Float32>("/planning/uncertainty", 10);
}

void ExplorationFSM::FSMCallback() {
  RCLCPP_DEBUG_STREAM(node_->get_logger(),  "--------------------START OF CALLBACK--------------------");

  // Current time in sec
  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[FSM] Current time: %.2lf", node_->now().seconds());

  // if (state_ == INIT || state_ == WAIT_TRIGGER || state_ == FINISH)
  //   RCLCPP_INFO(node_->get_logger(), "[FSM] Current state: %s", fd_->state_str_[int(state_)].c_str());
  // else
  //   RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[FSM] Current state: %s", fd_->state_str_[int(state_)].c_str());

  // Log current state and calculate avg callback rate
  static int cnt_callback = 0;
  static rclcpp::Time start_time = node_->now();
  static rclcpp::Time last_time = node_->now();
  double dt = (node_->now() - start_time).seconds();
  RCLCPP_DEBUG_STREAM(node_->get_logger(),  "[FSM] Current state: " << fd_->state_str_[int(state_)]
            << ", callback rate: " << cnt_callback / dt << "Hz");
  if (dt > 1.0) {
    start_time = node_->now();
    cnt_callback = 0;
  }
  cnt_callback++;

  switch (state_) {
  case INIT: {
    // Wait for odometry ready
    if (!fd_->have_odom_) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[FSM] No odom");
      return;
    }
    if ((node_->now() - fd_->fsm_init_time_).seconds() < 2.0) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[FSM] Initializing");
      return;
    }
    // Go to wait trigger when odom is ok
    RCLCPP_WARN(node_->get_logger(), "[FSM] Receive odom from topic %s", odom_sub_->get_topic_name());
    RCLCPP_WARN_STREAM(node_->get_logger(),  "[FSM] Receive odom from topic " << odom_sub_->get_topic_name());
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    // Update frontiers(in callback) and hgrid when waiting for trigger
    if (frontier_ready_)
      expl_manager_->initializeHierarchicalGrid(fd_->odom_pos_, fd_->odom_vel_);

    visualize();

    break;
  }

  case FINISH: {
    RCLCPP_INFO_ONCE(node_->get_logger(), "[FSM] Exploration finished");

    std_msgs::msg::Int32 replan_msg;
    replan_msg.data = 2;
    replan_pub_->publish(replan_msg);

    static bool clear_vis = false;
    if (!clear_vis) {
      auto ed_ = expl_manager_->ed_;
      ed_->n_points_.clear();
      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      ed_->refined_tour_.clear();
      ed_->grid_tour_.clear();
      ed_->grid_tour2_.clear();
      ed_->grid_tour3_.clear();
      visualize();
      clear_vis = true;

      double map_coverage = planner_manager_->map_server_->getMapCoverage();
      expl_manager_->ed_->finish_time_ = node_->now();
      RCLCPP_INFO(node_->get_logger(), "[FSM] Exploration finished. Start time: %.2lf, End time: %.2lf, Duration: %.2lf, "
               "Coverage: %.2lf",
               expl_manager_->ed_->start_time_.seconds(), expl_manager_->ed_->finish_time_.seconds(),
               (expl_manager_->ed_->finish_time_ - expl_manager_->ed_->start_time_).seconds(),
               map_coverage);

      if (false) {
        // Save times to /home/eason/workspace/exploration_ws/times.csv
        string output_file = "/home/eason/workspace/exploration_ws/times.csv";
        ofstream fout(output_file, ios::app);
        if (!fout.is_open()) {
          RCLCPP_ERROR(node_->get_logger(), "[FSM] Cannot open file %s", output_file.c_str());
          return;
        }

        // std::vector<double> frontier_times_;
        // std::vector<double> space_decomp_times_;
        // std::vector<double> connectivity_graph_times_;
        // std::vector<std::pair<double, double>> cp_times_;
        // std::vector<std::pair<double, double>> sop_times_;

        for (int i = 0; i < expl_manager_->ee_->frontier_times_.size(); ++i) {
          fout << expl_manager_->ee_->frontier_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->total_times_.size(); ++i) {
          fout << expl_manager_->ee_->total_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->space_decomp_times_.size(); ++i) {
          fout << expl_manager_->ee_->space_decomp_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->connectivity_graph_times_.size(); ++i) {
          fout << expl_manager_->ee_->connectivity_graph_times_[i] << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->cp_times_.size(); ++i) {
          fout << expl_manager_->ee_->cp_times_[i].first << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->cp_times_.size(); ++i) {
          fout << expl_manager_->ee_->cp_times_[i].second << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->sop_times_.size(); ++i) {
          fout << expl_manager_->ee_->sop_times_[i].first << ", ";
        }
        fout << endl;
        for (int i = 0; i < expl_manager_->ee_->sop_times_.size(); ++i) {
          fout << expl_manager_->ee_->sop_times_[i].second << ", ";
        }
        fout << endl;
        fout.close();

        // calculate mean and std and max and min for each times vector
        auto calc_mean_std = [](const std::vector<double> &vec, double &mean, double &std,
                                double &max, double &min) {
          mean = 0.0;
          std = 0.0;
          max = -1e10;
          min = 1e10;
          for (auto v : vec) {
            mean += v;
            max = std::max(max, v);
            min = std::min(min, v);
          }
          mean /= vec.size();
          for (auto v : vec) {
            std += (v - mean) * (v - mean);
          }
          std = std::sqrt(std / vec.size());
        };
        auto calc_mean_std2 = [](const std::vector<std::pair<double, double>> &vec,
                                pair<double, double> &mean, pair<double, double> &std,
                                pair<double, double> &max, pair<double, double> &min) {
          mean.first = 0.0;
          mean.second = 0.0;
          std.first = 0.0;
          std.second = 0.0;
          max.first = -1e10;
          max.second = -1e10;
          min.first = 1e10;
          min.second = 1e10;
          for (auto v : vec) {
            mean.first += v.first;
            mean.second += v.second;
            max.first = std::max(max.first, v.first);
            max.second = std::max(max.second, v.second);
            min.first = std::min(min.first, v.first);
            min.second = std::min(min.second, v.second);
          }
          mean.first /= vec.size();
          mean.second /= vec.size();
          for (auto v : vec) {
            std.first += (v.first - mean.first) * (v.first - mean.first);
            std.second += (v.second - mean.second) * (v.second - mean.second);
          }
          std.first = std::sqrt(std.first / vec.size());
          std.second = std::sqrt(std.second / vec.size());
        };

        double mean, std, max, min;
        calc_mean_std(expl_manager_->ee_->frontier_times_, mean, std, max, min);
        RCLCPP_INFO(node_->get_logger(), "[FSM] Frontier times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean, std, max,
                min);
        calc_mean_std(expl_manager_->ee_->total_times_, mean, std, max, min);
        RCLCPP_INFO(node_->get_logger(), "[FSM] Total times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean, std, max,
                min);
        calc_mean_std(expl_manager_->ee_->space_decomp_times_, mean, std, max, min);
        RCLCPP_INFO(node_->get_logger(), "[FSM] Space decomp times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean, std,
                max, min);
        calc_mean_std(expl_manager_->ee_->connectivity_graph_times_, mean, std, max, min);
        RCLCPP_INFO(node_->get_logger(), "[FSM] Connectivity graph times: mean %.6lf, std %.6lf, max %.6lf, min %.6lf", mean,
                std, max, min);

        pair<double, double> mean2, std2, max2, min2;
        calc_mean_std2(expl_manager_->ee_->cp_times_, mean2, std2, max2, min2);
        RCLCPP_INFO(node_->get_logger(),
            "[FSM] CP times: mean %.6lf, %.6lf, std %.6lf, %.6lf, max %.6lf, %.6lf, min %.6lf, %.6lf",
            mean2.first, mean2.second, std2.first, std2.second, max2.first, max2.second, min2.first,
            min2.second);
        calc_mean_std2(expl_manager_->ee_->sop_times_, mean2, std2, max2, min2);
        RCLCPP_INFO(node_->get_logger(), "[FSM] SOP times: mean %.6lf, %.6lf, std %.6lf, %.6lf, max %.6lf, %.6lf, min %.6lf, "
                "%.6lf",
                mean2.first, mean2.second, std2.first, std2.second, max2.first, max2.second,
                min2.first, min2.second);
      }
    }

    // if (node_->now() - expl_manager_->ed_->finish_time_ > rclcpp::Duration::from_seconds(5.0))
    //   transitState(RTB, "FSM");

    break;
  }

  case RTB: {
    // TODO
    RCLCPP_INFO_ONCE(node_->get_logger(), "[FSM] Return to base");
    break;
  }

  case PLAN_TRAJ: {
    if (waiting_for_stop_) {
      if (last_odom_stamp_ < stop_requested_time_ || fd_->odom_vel_.norm() > 0.05 ||
          fd_->odom_angular_speed_ > 1e-3) break;
      waiting_for_stop_ = false;
    }
    int cur_cell_id, cur_center_id;
    expl_manager_->hierarchical_grid_->getLayerPositionCellCenterId(0, fd_->odom_pos_, cur_cell_id,
                                                                    cur_center_id);
    RCLCPP_INFO(node_->get_logger(), "[FSM] Current cell id: %d, center id: %d", cur_cell_id, cur_center_id);

    // if (cur_cell_id == -1 || cur_center_id == -1) {
    //   CHECK(false) << "Invalid current cell id or center id";
    // }

    double map_coverage = planner_manager_->map_server_->getMapCoverage();
    RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM] Exploration planning. Start time: "
                    << expl_manager_->ed_->start_time_.seconds() << ", Current time: " << node_->now().seconds()
                    << ", Duration: " << (node_->now() - expl_manager_->ed_->start_time_).seconds()
                    << ", Coverage: " << map_coverage);

    if (fd_->static_state_) {
      // Plan from static state (hover)
      fd_->start_pos_ = fd_->odom_pos_;
      fd_->start_vel_ = fd_->odom_vel_;
      fd_->start_acc_.setZero();
      fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      trajectory_start_time_ = node_->now() + rclcpp::Duration::from_seconds(fp_->replan_duration_);
    } else {
      // Replan from non-static state, starting from 'replan_time' seconds later
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_r = (time_now - info->start_time_).seconds() + fp_->replan_duration_;
      if (t_r > info->duration_) {
        t_r = info->duration_;
      }
      trajectory_start_time_ = time_now + rclcpp::Duration::from_seconds(fp_->replan_duration_);
      fd_->start_pos_ = info->position_traj_.evaluateDeBoorT(t_r);
      fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
      fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
      fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
    }

    if (false) {
      // Print current states
      RCLCPP_INFO(node_->get_logger(), "[FSM] Current pos: %.2lf, %.2lf, %.2lf", fd_->odom_pos_[0], fd_->odom_pos_[1],
               fd_->odom_pos_[2]);
      RCLCPP_INFO(node_->get_logger(), "[FSM] Current vel: %.2lf, %.2lf, %.2lf", fd_->odom_vel_[0], fd_->odom_vel_[1],
               fd_->odom_vel_[2]);
      RCLCPP_INFO(node_->get_logger(), "[FSM] Current yaw: %.2lf", fd_->odom_yaw_);

      RCLCPP_DEBUG_STREAM(node_->get_logger(),  "[FSM] Current pos: " << fd_->odom_pos_.transpose()
                << ", vel: " << fd_->odom_vel_.transpose() << ", yaw: " << fd_->odom_yaw_);

      // Print start states
      RCLCPP_INFO(node_->get_logger(), "[FSM] Start pos: %.2lf, %.2lf, %.2lf", fd_->start_pos_[0], fd_->start_pos_[1],
               fd_->start_pos_[2]);
      RCLCPP_INFO(node_->get_logger(), "[FSM] Start vel: %.2lf, %.2lf, %.2lf", fd_->start_vel_[0], fd_->start_vel_[1],
               fd_->start_vel_[2]);
      RCLCPP_INFO(node_->get_logger(), "[FSM] Start acc: %.2lf, %.2lf, %.2lf", fd_->start_acc_[0], fd_->start_acc_[1],
               fd_->start_acc_[2]);
      RCLCPP_INFO(node_->get_logger(), "[FSM] Start yaw: %.2lf, %.2lf, %.2lf", fd_->start_yaw_[0], fd_->start_yaw_[1],
               fd_->start_yaw_[2]);

      RCLCPP_DEBUG_STREAM(node_->get_logger(),  "[FSM] Start pos: " << fd_->start_pos_.transpose()
                << ", vel: " << fd_->start_vel_.transpose()
                << ", acc: " << fd_->start_acc_.transpose()
                << ", yaw: " << fd_->start_yaw_.transpose());

      for (int i = 0; i < 3; ++i) {
        if (abs(fd_->start_vel_[i]) > planner_manager_->pp_.max_vel_) {
          RCLCPP_WARN(node_->get_logger(), "[FSM] Start vel too high: %.2lf", fd_->start_vel_[i]);
        }

        if (abs(fd_->start_acc_[i]) > planner_manager_->pp_.max_acc_) {
          RCLCPP_WARN(node_->get_logger(), "[FSM] Start acc too high: %.2lf", fd_->start_acc_[i]);
        }
      }
    }

    // Keep the checked old trajectory until replacement or an explicit stop.
    // A fixed 0.5 s truncation disagrees with a measured longer planning horizon.

    // Exploration plannner main function
    int res = callExplorationPlanner();
    if (res == SUCCEED) {
      transitState(PUB_TRAJ, "FSM");
    } else if (res == FAIL) { // Keep trying to replan
      stopAndWaitForOdometry();
      RCLCPP_WARN(node_->get_logger(), "[FSM] Plan fail");
    } else if (res == NO_GRID) {
      fd_->static_state_ = true;
      RCLCPP_WARN(node_->get_logger(), "[FSM] Finish exploration: No grid");

      CHECK_EQ(expl_manager_->ed_->frontiers_.size(), 0)
          << "Frontier size: " << expl_manager_->ed_->frontiers_.size() << ", but no grid";

      transitState(FINISH, "FSM");
    }

    visualize();

    break;
  }

  case PUB_TRAJ: {
    const double publication_lateness = (node_->now() - trajectory_start_time_).seconds();
    if (publication_lateness > 0.05) {
      RCLCPP_WARN(node_->get_logger(), "[FSM] Discarding plan delayed before publication");
      publication_margin_ += publication_lateness + 0.05;
      fp_->replan_duration_ += publication_lateness + 0.05;
      expl_manager_->ep_->replan_duration_ = fp_->replan_duration_;
      stopAndWaitForOdometry();
      transitState(PLAN_TRAJ, "late publication");
      break;
    }
    bool safe = planner_manager_->checkTrajCollision();
    // if (!safe) {
    //   RCLCPP_ERROR(node_->get_logger(), "[FSM] Replan: collision detected on the trajectory before publishing");
    //   fd_->static_state_ = true;
    //   transitState(PLAN_TRAJ, "FSM");
    //   break;
    // }

    if (!safe) {
      RCLCPP_ERROR(node_->get_logger(), "[FSM] Collision detected on the trajectory before publishing");
      const double goal_yaw = planner_manager_->local_data_.end_yaw_;

      planner_manager_->local_data_.position_traj_ =
          planner_manager_->local_data_.init_traj_bspline_;
      planner_manager_->local_data_.velocity_traj_ =
          planner_manager_->local_data_.position_traj_.getDerivative();
      planner_manager_->local_data_.acceleration_traj_ =
          planner_manager_->local_data_.velocity_traj_.getDerivative();
      planner_manager_->local_data_.duration_ =
          planner_manager_->local_data_.position_traj_.getTimeSum();
      planner_manager_->local_data_.start_pos_ =
          planner_manager_->local_data_.position_traj_.evaluateDeBoorT(0.0);
      planner_manager_->local_data_.end_pos_ =
          planner_manager_->local_data_.position_traj_.evaluateDeBoorT(
              planner_manager_->local_data_.duration_);
      const bool yaw_safe = planner_manager_->planExplorationYawWaypointsTraj(
          fd_->start_yaw_(0), fd_->start_yaw_(1), goal_yaw, {}, {}, fd_->start_yaw_(2));

      bool safe_init_path = planner_manager_->checkTrajCollision();

      if (!safe_init_path || !yaw_safe) {
        RCLCPP_ERROR(node_->get_logger(), "[FSM] Replan: collision also detected on the initial trajectory");

        stopAndWaitForOdometry();
        transitState(PLAN_TRAJ, "FSM");
        break;
      }

      RCLCPP_WARN(node_->get_logger(), "[FSM] Replacing the trajectory with the safe initial trajectory");
    }

    // Preserve the planner's time parameterization and derivative consistency.
    // Compressing knots here bypassed dynamic limits and left stale derivatives.

    falcon_interfaces::msg::Bspline bspline;
    auto info = &planner_manager_->local_data_;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;

    double dt = (node_->now() - rclcpp::Time(fd_->newest_traj_.start_time, RCL_ROS_TIME)).seconds();
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM] Traj dt: " << dt);
    if (dt > 0.0) {
      bspline_pub_->publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      transitState(EXEC_TRAJ, "FSM");

      // traj_lengthened = false;

      // thread vis_thread(&ExplorationFSM::visualize, this);
      // vis_thread.detach();
    }
    break;
  }

  case EXEC_TRAJ: {
    auto tn = node_->now();
    // Check whether replan is needed
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (tn - info->start_time_).seconds();

    // if (!fd_->go_back_) {
    bool need_replan = false;
    int replan_type = 0;
    if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
      // Replan if frontier cluster is covered with some percentage
      RCLCPP_WARN(node_->get_logger(), "[FSM] Replan: cluster covered");
      need_replan = true;
      replan_type = 1;
    } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
      // Replan if traj is almost fully executed
      RCLCPP_WARN(node_->get_logger(), "[FSM] Replan: traj fully executed");
      need_replan = true;
      replan_type = 2;
    } else if (t_cur > fp_->replan_thresh3_) {
      // Replan after some time
      RCLCPP_WARN(node_->get_logger(), "[FSM] Replan: periodic call");
      need_replan = true;
      replan_type = 3;
    }

    if (need_replan) {
      {
        // Log replan type
        string replan_type_str;
        switch (replan_type) {
        case 0: {
          replan_type_str = "no replan";
          break;
        }
        case 1: {
          replan_type_str = "cluster covered";
          break;
        }
        case 2: {
          replan_type_str = "traj fully executed";
          break;
        }
        case 3: {
          replan_type_str = "periodic call";
          break;
        }
        }
        RCLCPP_DEBUG_STREAM(node_->get_logger(),  "[FSM] Replan type: " << replan_type_str);

        // Visualize replan type in Rviz
        visualization_->drawText(Eigen::Vector3d(12, 0, 0), replan_type_str, 1,
                                 PlanningVisualization::Color::Black(), "replan_type", 0,
                                 PlanningVisualization::PUBLISHER::DEBUG);
      }

      if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
        // Update frontier and plan new motion
        visualize();

        transitState(PLAN_TRAJ, "FSM");

        // Use following code can debug the planner step by step
        // transitState(WAIT_TRIGGER, "FSM");
        // fd_->static_state_ = true;

      } else {
        // No frontier detected, finish exploration
        transitState(FINISH, "FSM");
        RCLCPP_WARN(node_->get_logger(), "[FSM] Finish exploration: No frontier detected");
        // clearVisMarker();
        // visualize();
      }
    }

    break;
  }
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(),  "[FSM] Callback time: " << (node_->now() - last_time).seconds() << "s");
  RCLCPP_DEBUG_STREAM(node_->get_logger(),  "--------------------END OF CALLBACK--------------------");
  RCLCPP_DEBUG_STREAM(node_->get_logger(),  "");

  last_time = node_->now();
}

int ExplorationFSM::callExplorationPlanner() {
  const auto plan_start_time = std::chrono::steady_clock::now();
  int res = expl_manager_->planExploreMotionHGrid(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_,
                                                  fd_->start_yaw_);
  const double plan_total_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - plan_start_time).count();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[FSM] Exploration planner total time: "
                  << std::setprecision(4) << plan_total_time * 1000.0
                  << "ms, replan duration: " << fp_->replan_duration_ * 1000.0 << "ms");
  if (plan_total_time > fp_->replan_duration_) RCLCPP_WARN(node_->get_logger(), "[FSM] Planning exceeded replan duration");

  // Dynamic replan duration
  if (plan_total_time > fp_->replan_duration_slow_) {
    fp_->replan_duration_ = fp_->replan_duration_slow_;
  } else if (plan_total_time < fp_->replan_duration_fast_) {
    fp_->replan_duration_ = fp_->replan_duration_fast_;
  } else {
    fp_->replan_duration_ = fp_->replan_duration_default_;
  }
  // The configured slow horizon is a baseline, not a cap on measured CPU time.
  fp_->replan_duration_ = std::max(fp_->replan_duration_, plan_total_time + publication_margin_);
  expl_manager_->ep_->replan_duration_ = fp_->replan_duration_;

  if (res == SUCCEED) {
    if ((node_->now() - trajectory_start_time_).seconds() > 0.05) {
      RCLCPP_WARN(node_->get_logger(), "[FSM] Discarding late plan; stop and retry from current odometry");
      return FAIL;
    }
    auto info = &planner_manager_->local_data_;
    // ROS_ERROR_COND((node_->now() - time_r).seconds() < -0.1,
    //                "[FSM] replan_time set too large, please decrease it. diff: %lf",
    //                (node_->now() - time_r).seconds());
    // ROS_ERROR_COND((node_->now() - time_r).seconds() > 0.1,
    //                "[FSM] replan_time set too small, please increase it. diff: %lf",
    //                (node_->now() - time_r).seconds());
    // info->start_time_ = (node_->now() - time_r).seconds() > 0 ? node_->now() : time_r;
    info->start_time_ = trajectory_start_time_;
  }

  return res;
}

void ExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->map_server_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3),
  // "updated_box", 0, 4);

  // New frontier visualization, using pointcloud instead of marker
  double map_resolution = planner_manager_->map_server_->getResolution();
  if (map_resolution > 0.1) {
    visualization_->drawFrontierPointcloudHighResolution(ed_ptr->frontiers_, map_resolution);
    visualization_->drawDormantFrontierPointcloudHighResolution(ed_ptr->dormant_frontiers_,
                                                                map_resolution);
    visualization_->drawTinyFrontierPointcloudHighResolution(ed_ptr->tiny_frontiers_,
                                                             map_resolution);
  } else {
    visualization_->drawFrontierPointcloud(ed_ptr->frontiers_);
    visualization_->drawDormantFrontierPointcloud(ed_ptr->dormant_frontiers_);
    visualization_->drawTinyFrontierPointcloud(ed_ptr->tiny_frontiers_);
  }

  vector<int> visib_nums;
  expl_manager_->frontier_finder_->getDormantFrontiersVisibleNums(visib_nums);

  static std::vector<int> last_frontier_ids;
  // cout << "Clear previous frontier ids" << endl;
  for (auto i : last_frontier_ids) {
    visualization_->drawText(Eigen::Vector3d::Zero(), "", 0.5, PlanningVisualization::Color::Red(),
                             "frontier_id", i, PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_frontier_ids.clear();
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawText(ed_ptr->averages_[i], to_string(i), 0.5,
                             PlanningVisualization::Color::Red(), "frontier_id", i,
                             PlanningVisualization::PUBLISHER::FRONTIER);
    last_frontier_ids.push_back(i);
  }

  static std::vector<int> last_dormant_frontier_ids;
  // cout << "Clear previous dormant frontier ids" << endl;
  for (auto i : last_dormant_frontier_ids) {
    visualization_->drawText(Eigen::Vector3d::Zero(), "", 0.5, PlanningVisualization::Color::Red(),
                             "frontier_visble_num", i, PlanningVisualization::PUBLISHER::FRONTIER);
  }
  last_dormant_frontier_ids.clear();
  for (int i = 0; i < ed_ptr->dormant_frontiers_.size(); ++i) {
    visualization_->drawText(ed_ptr->dormant_frontiers_[i][0], to_string(visib_nums[i]), 1.0,
                             PlanningVisualization::Color::Red(), "frontier_visble_num", i,
                             PlanningVisualization::PUBLISHER::FRONTIER);
    last_dormant_frontier_ids.push_back(i);
  }

  visualization_->drawLines(ed_ptr->grid_tour2_, 0.1, PlanningVisualization::Color::Blue2(),
                            "hgrid_tour_mc", 0, PlanningVisualization::PUBLISHER::HGRID);

  // Draw global top viewpoints info
  visualization_->drawSpheres(ed_ptr->points_, 0.2, PlanningVisualization::Color::DeepGreen(),
                              "points", 0, PlanningVisualization::PUBLISHER::VIEWPOINT);

  visualization_->drawLines(ed_ptr->views1_, ed_ptr->views2_, 0.02,
                            PlanningVisualization::Color::Black(), "view_fov", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw a line from frontier top view point with its view direction
  visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05,
                            PlanningVisualization::Color::Yellow(), "frontier_view", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);
  // Draw a line from frontier top view point to frontier average center
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03,
  //                           PlanningVisualization::Color::DeepGreen(), "point_average", 0,
  //                           PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw local refined viewpoints info
  visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, PlanningVisualization::Color::Blue(),
                              "refined_pts", 0, 6);

  visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
                            PlanningVisualization::Color::LightBlue(), "refined_view", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawLines(ed_ptr->refined_tour_, 0.07, PlanningVisualization::Color::DeepBlue(),
                            "refined_tour", 0, 6);

  visualization_->drawLines(ed_ptr->path_next_goal_, 0.07, PlanningVisualization::Color::DeepBlue(),
                            "path_next_goal", 0, 6);

  // Draw refined view FOV
  visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.02,
                            PlanningVisualization::Color::Black(), "refined_view_fov", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw a line to show the pair of the original view point and refined view point
  visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.02,
                            PlanningVisualization::Color::Yellow(), "refine_pair", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);

  // visualization_->drawLines(ed_ptr->n_views1_, ed_ptr->n_views2_, 0.02,
  //                           PlanningVisualization::Color::Black(), "n_views_fov", 0,
  //                           PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw sampled viewpoints
  for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    visualization_->drawSpheres(
        ed_ptr->n_points_[i], 0.2,
        visualization_->getColor(double(ed_ptr->refined_ids_[i]) / ed_ptr->frontiers_.size()),
        "n_points", i, PlanningVisualization::PUBLISHER::VIEWPOINT);
  // Remove markers if this time has less #frontiers than previous time
  for (int i = ed_ptr->n_points_.size(); i < 20; ++i)
    visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i,
                                PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw next goal position
  Eigen::Quaterniond next_q;
  next_q = Eigen::AngleAxisd(ed_ptr->next_yaw_, Eigen::Vector3d::UnitZ());
  visualization_->drawPose(ed_ptr->next_goal_, next_q, "next_goal", 0);
  // visualization_->drawSpheres({ed_ptr->next_goal_}, 1.0,
  //                             PlanningVisualization::Color::TealTransparent(), "next_goal", 0,
  //                             PlanningVisualization::PUBLISHER::VIEWPOINT);

  // Draw trajectory
  // visualization_->drawPolynomialTraj(info->init_traj_poly_, 0.1,
  // PlanningVisualization::Color::Red(), 0); visualization_->drawPolynomialTraj(info->init_traj2_,
  // 0.1, PlanningVisualization::Color::Red(), 1);
  if (info->position_traj_.getKnot().size() > 0) {
    visualization_->drawBspline(planner_manager_->local_data_.init_traj_bspline_, 0.05,
                                PlanningVisualization::Color::DeepGreen(), true, 0.1,
                                PlanningVisualization::Color::DeepGreen(), 0);
    visualization_->drawBspline(planner_manager_->local_data_.position_traj_, 0.05,
                                PlanningVisualization::Color::DeepBlue(), true, 0.1,
                                PlanningVisualization::Color::DeepBlue(), 1);
    // visualization_->drawBsplineWithVelocity(info->position_traj_, info->velocity_traj_);
  }

  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1,
  // 6);

  // expl_manager_->hierarchical_grid_->publishGridsLayer();
  // expl_manager_->hierarchical_grid_->publishGridsConnectivityLayer();

  // Draw frontier first pc
  // vector<Vector2d> first_pcs;
  // vector<Position> first_pcs_3d;
  // expl_manager_->frontier_finder_->getFrontierFirstPCs(first_pcs);

  // first_pcs_3d.resize(first_pcs.size());
  // for (int i = 0; i < first_pcs.size(); ++i) {
  //   first_pcs_3d[i] = ed_ptr->averages_[i] + Vector3d(first_pcs[i](0), first_pcs[i](1), 0);
  // }

  // visualization_->drawLines(ed_ptr->averages_, first_pcs_3d, 0.02,
  //                           PlanningVisualization::Color::White(), "frontier_first_pc", 0,
  //                           PlanningVisualization::PUBLISHER::FRONTIER);
}

void ExplorationFSM::clearVisMarker() {
  visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);
  visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void ExplorationFSM::frontierCallback() {
  static int delay = 0;
  if (++delay < 5)
    return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();

    Position update_bbox_min, update_bbox_max;
    ft->getUpdateBBox(update_bbox_min, update_bbox_max);
    ed->update_bbox_min_ = update_bbox_min;
    ed->update_bbox_max_ = update_bbox_max;

    ft->getFrontiers(ed->frontiers_);
    ft->getDormantFrontiers(ed->dormant_frontiers_);
    ft->getTinyFrontiers(ed->tiny_frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);
    ft->getTopViewpointsInfo(fd_->odom_pos_, ed->points_, ed->yaws_, ed->averages_);

    double map_resolution = planner_manager_->map_server_->getResolution();
    if (map_resolution > 0.1) {
      visualization_->drawFrontierPointcloudHighResolution(ed->frontiers_, map_resolution);
      visualization_->drawDormantFrontierPointcloudHighResolution(ed->dormant_frontiers_,
                                                                  map_resolution);
      visualization_->drawTinyFrontierPointcloudHighResolution(ed->tiny_frontiers_, map_resolution);
    } else {
      visualization_->drawFrontierPointcloud(ed->frontiers_);
      visualization_->drawDormantFrontierPointcloud(ed->dormant_frontiers_);
      visualization_->drawTinyFrontierPointcloud(ed->tiny_frontiers_);
    }

    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawText(ed->averages_[i], to_string(i), 0.5,
                               PlanningVisualization::Color::Red(), "frontier_id", i,
                               PlanningVisualization::PUBLISHER::FRONTIER);
      visualization_->drawSpheres(ed->points_, 0.2, PlanningVisualization::Color::DeepGreen(),
                                  "points", 0, PlanningVisualization::PUBLISHER::VIEWPOINT);
    }

    if (ed->frontiers_.size() > 0) {
      frontier_ready_ = true;
    }
  }

  if (expl_manager_->ep_->auto_start_ && frontier_ready_ && state_ == WAIT_TRIGGER) {
    fd_->triggered_ = true;
    transitState(PLAN_TRAJ, "frontierCallback");

    expl_manager_->ed_->start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "[FSM] Exploration start time: %lf", expl_manager_->ed_->start_time_.seconds());
  }
}

void ExplorationFSM::triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  if (state_ != WAIT_TRIGGER)
    return;

  // Can be only triggered after frontier is generated
  if (!frontier_ready_)
    return;

  fd_->triggered_ = true;
  transitState(PLAN_TRAJ, "triggerCallback");

  expl_manager_->ed_->start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "[FSM] Exploration start time: %lf", expl_manager_->ed_->start_time_.seconds());
}

void ExplorationFSM::safetyCallback() {
  // Check if current pos is in occupied area
  // CHECK(planner_manager_->map_server_->getOccupancy(fd_->odom_pos_) !=
  //       voxel_mapping::OccupancyType::OCCUPIED)
  //     << "Exploration failed as current pos is in occupied area. Please restart the program.";

  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    bool safe = planner_manager_->checkTrajCollision();
    if (!safe) {
      RCLCPP_ERROR(node_->get_logger(), "[FSM] Collision detected on the trajectory! Replanning...");
      stopAndWaitForOdometry();
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void ExplorationFSM::stopAndWaitForOdometry() {
  std_msgs::msg::Int32 stop;
  stop.data = 1;
  replan_pub_->publish(stop);
  stop_requested_time_ = node_->now();
  waiting_for_stop_ = true;
  fd_->static_state_ = true;
}

void ExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  const auto &p = msg->pose.pose.position;
  const auto &v = msg->twist.twist.linear;
  const auto &angular = msg->twist.twist.angular;
  const auto &q = msg->pose.pose.orientation;
  const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!Eigen::Vector3d(p.x,p.y,p.z).allFinite() || !Eigen::Vector3d(v.x,v.y,v.z).allFinite() ||
      !Eigen::Vector3d(angular.x,angular.y,angular.z).allFinite() ||
      !orientation.coeffs().allFinite() || orientation.norm() < 1e-6 ||
      expl_manager_->hierarchical_grid_->getLayerCellId(0, Eigen::Vector3d(p.x,p.y,p.z)) < 0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Ignoring invalid or out-of-map odometry");
    return;
  }
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  last_odom_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;
  fd_->odom_angular_speed_ = Eigen::Vector3d(angular.x,angular.y,angular.z).norm();

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;
  fd_->odom_orient_.normalize();

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = node_->now();
  }

  if (state_ == INIT || state_ == WAIT_TRIGGER)
    return;

  static int delay = 0;
  static double accum_dist = 0;
  if (++delay < 10)
    return;
  else {
    delay = 0;
    if (expl_manager_->ed_->trajecotry_.size() > 0)
      accum_dist += (fd_->odom_pos_ - expl_manager_->ed_->trajecotry_.back()).norm();
    expl_manager_->ed_->trajecotry_.push_back(fd_->odom_pos_);
  }
}

void ExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  RCLCPP_WARN_STREAM(node_->get_logger(), "[FSM] "
                  << "Transit state from " + fd_->state_str_[pre_s] + " to " +
                         fd_->state_str_[int(new_state)] + " by " + pos_call);
  RCLCPP_WARN_STREAM(node_->get_logger(),  "[FSM] "
               << "ROS Time: " << node_->now().seconds()
               << " Transit state from " + fd_->state_str_[pre_s] + " to " +
                      fd_->state_str_[int(new_state)] + " by " + pos_call);
}
} // namespace fast_planner
