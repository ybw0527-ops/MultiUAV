#ifndef FALCON_EXPLORATION_PREPROCESSING_CONNECTIVITY_GRAPH_H_
#define FALCON_EXPLORATION_PREPROCESSING_CONNECTIVITY_GRAPH_H_

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "exploration_types.h"

namespace fast_planner {

struct ConnectivityEdge {
  using Ptr = std::shared_ptr<ConnectivityEdge>;
  using ConstPtr = std::shared_ptr<const ConnectivityEdge>;

  enum class TYPE { UNKNOWN, FREE, PORTAL };

  TYPE type_{TYPE::UNKNOWN};
  int id1_{-1};
  int id2_{-1};
  double cost_{std::numeric_limits<double>::infinity()};
  std::vector<Position> path_;
};

struct ConnectivityNode {
  using Ptr = std::shared_ptr<ConnectivityNode>;
  using ConstPtr = std::shared_ptr<const ConnectivityNode>;

  enum class TYPE { UNKNOWN, FREE };

  ConnectivityNode() = default;
  ConnectivityNode(int id, const Position &position, TYPE type)
      : type_(type), id_(id), pos_(position) {}

  void addNeighbor(int neighbor_id, double cost, ConnectivityEdge::TYPE type);
  void addNeighborWithPath(int neighbor_id, double cost, ConnectivityEdge::TYPE type,
                           const std::vector<Position> &path);
  void removeNeighbor(int neighbor_id);
  void clearNeighbors() { neighbors_.clear(); }

  TYPE type_{TYPE::UNKNOWN};
  int id_{-1};
  Position pos_{Position::Zero()};
  std::vector<ConnectivityEdge> neighbors_;
};

class ConnectivityGraph {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ConnectivityGraph>;
  using ConstPtr = std::shared_ptr<const ConnectivityGraph>;

  static constexpr double kUnreachableCost = 1000.0;
  static constexpr double kBlockedEdgeCost = 499.0;

  void addNode(const ConnectivityNode::Ptr &node);
  void removeNode(int id);
  void clearNodes();
  ConnectivityNode::Ptr getNode(int id) const;
  void getNodeNum(int &number) const;
  int getNodeNum() const;
  void getNodePositions(std::vector<Position> &positions) const;
  void getNodePositionsWithIDs(std::vector<Position> &positions,
                               std::vector<int> &ids) const;

  double searchConnectivityGraphBFS(int start_id, int goal_id, std::vector<int> &path) const;
  double searchConnectivityGraphAStar(int start_id, int goal_id,
                                      std::vector<int> &path) const;
  void getFullConnectivityGraph(std::vector<std::pair<Position, Position>> &edges,
                                std::vector<ConnectivityEdge::TYPE> &edge_types,
                                std::vector<double> &edge_costs) const;
  void getFullConnectivityGraphPath(std::vector<std::vector<Position>> &paths,
                                    std::vector<ConnectivityEdge::TYPE> &path_types) const;
  void findDisconnectedNodes(std::set<int> &disconnected_nodes) const;

  bool saveConnectivityGraph(const std::string &file_path) const;
  bool loadConnectivityGraph(const std::string &file_path);

private:
  bool edgeIsUsable(const ConnectivityEdge &edge) const;
  double pathCost(const std::vector<int> &path) const;

  std::unordered_map<int, ConnectivityNode::Ptr> nodes_;
};

} // namespace fast_planner

#endif // FALCON_EXPLORATION_PREPROCESSING_CONNECTIVITY_GRAPH_H_
