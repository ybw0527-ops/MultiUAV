#ifndef _ASTAR_H
#define _ASTAR_H

#include <iostream>
#include <functional>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxel_mapping/map_server.h"

using namespace voxel_mapping;

namespace fast_planner {

template <typename T> struct matrix_hash {
  std::size_t operator()(T const &matrix) const {
    size_t seed = 0;
    for (Eigen::Index i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class Astar {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  class AstarNode {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef AstarNode *Ptr; // Use raw pointer for better performance

    Eigen::Vector3d position;
    Eigen::Vector3i index;
    double g_score, f_score;
    Ptr parent;

    AstarNode() { parent = NULL; }
    ~AstarNode(){};
  };

  class AstarNodeComparator {
  public:
    bool operator()(AstarNode::Ptr node1, AstarNode::Ptr node2) {
      return node1->f_score > node2->f_score;
    }
  };

  typedef std::shared_ptr<Astar> Ptr;
  typedef std::shared_ptr<const Astar> ConstPtr;

  enum class PROFILE { DEFAULT, COARSE, COARSE2, MEDIUM, FINE };
  enum class MODE {
    FREE_ONLY,
    FREE_ONLY_BBOX,
    FREE_UNKNOWN,
    FREE_UNKNOWN_BBOX,
    UNKNOWN_ONLY,
    UNKNOWN_ONLY_BBOX
  };
  enum { REACH_END = 1, NO_PATH = 2 };

  struct Config {
    double resolution_{0.5}, resolution_inverse_{2.0}; // astar search step
    double max_search_time_{0.01};                    // astar search time limit
    double lambda_heuristic_{1.0};                    // weight for heuristic
    int allocate_num_{100000};                        // number of nodes in the pool

    double epsilon_{1e-4}, tie_breaker_{1.0001};

    // profile settings
    double default_resolution_{0.5};
    double default_max_search_time_{0.01};
    double coarse_resolution_{1.0};
    double coarse_max_search_time_{0.001};
    double coarse2_resolution_{1.0};
    double coarse2_max_search_time_{0.01};
    double medium_resolution_{1.0};
    double medium_max_search_time_{0.1};
    double fine_resolution_{0.1};
    double fine_max_search_time_{0.1};

    // map settings
    Eigen::Vector3d map_origin_{Eigen::Vector3d::Zero()};

    bool verbose_{false};

    void print() {
      std::cout
          << "|---------------------------------- Astar Config ----------------------------------|"
          << std::endl;
      std::cout << "Resolution: " << resolution_ << std::endl;
      std::cout << "Max search time: " << max_search_time_ << std::endl;
      std::cout << "Lambda heuristic: " << lambda_heuristic_ << std::endl;
      std::cout << "Epsilon: " << epsilon_ << std::endl;
      std::cout << "Tie breaker: " << tie_breaker_ << std::endl;
      std::cout << "Default resolution: " << default_resolution_ << std::endl;
      std::cout << "Default max search time: " << default_max_search_time_ << std::endl;
      std::cout << "Coarse resolution: " << coarse_resolution_ << std::endl;
      std::cout << "Coarse max search time: " << coarse_max_search_time_ << std::endl;
      std::cout << "Medium resolution: " << medium_resolution_ << std::endl;
      std::cout << "Medium max search time: " << medium_max_search_time_ << std::endl;
      std::cout << "Fine resolution: " << fine_resolution_ << std::endl;
      std::cout << "Fine max search time: " << fine_max_search_time_ << std::endl;
      std::cout << "Map origin: " << map_origin_.transpose() << std::endl;
      std::cout << "Verbose: " << verbose_ << std::endl;
      std::cout
          << "|---------------------------------------------------------------------------------|"
          << std::endl;
    }
  };

  Astar();
  ~Astar();

  void init(rclcpp::Node &node, const MapServer::Ptr &map_server);
  void reset();

  int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const MODE &mode,
             const Position &bbox_min, const Position &bbox_max);

  int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
  int searchBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                 const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max);
  int searchUnknown(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
  int searchUnknownBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                        const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max);
  int searchUnknownOnlyBBox(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                            const Eigen::Vector3d &bbox_min, const Eigen::Vector3d &bbox_max);

  void setResolution(const double &res);
  void setMaxSearchTime(const double &t);
  void setProfile(const PROFILE &p);
  static double pathLength(const std::vector<Eigen::Vector3d> &path);

  void shortenPath(std::vector<Eigen::Vector3d> &path);

  // Optional local execution clearance for free-only search and shortening.
  // Global connectivity searches retain their existing occupancy semantics.
  using SegmentValidator = std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>;
  void setSegmentValidator(SegmentValidator validator) { segment_validator_=std::move(validator); }

  std::vector<Eigen::Vector3d> getPath();
  double getResolution();
  double getMaxSearchTime();

private:
  void backtrack(const AstarNode::Ptr &end_node, const Eigen::Vector3d &end);
  void posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);
  double getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
  double getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
  double getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

  Config config_;

  // main data structure
  std::vector<AstarNode::Ptr> path_node_pool_;
  int use_node_num_{0}, iter_num_{0};
  std::priority_queue<AstarNode::Ptr, std::vector<AstarNode::Ptr>, AstarNodeComparator> open_set_;
  std::unordered_map<Eigen::Vector3i, AstarNode::Ptr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
  std::vector<Eigen::Vector3d> path_nodes_;

  MapServer::Ptr map_server_;
  RayCaster::Ptr ray_caster_;
  rclcpp::Node *node_{nullptr};
  SegmentValidator segment_validator_;
};

} // namespace fast_planner

#endif
