#include "exploration_preprocessing/connectivity_graph.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <queue>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <unordered_set>

namespace fast_planner {
namespace {

bool blankLine(const std::string &line) {
  return line.find_first_not_of(" \t\r") == std::string::npos;
}

} // namespace

void ConnectivityNode::addNeighbor(const int neighbor_id, const double cost,
                                   const ConnectivityEdge::TYPE type) {
  addNeighborWithPath(neighbor_id, cost, type, {});
}

void ConnectivityNode::addNeighborWithPath(const int neighbor_id, const double cost,
                                           const ConnectivityEdge::TYPE type,
                                           const std::vector<Position> &path) {
  if (!std::isfinite(cost) || cost < 0.0)
    throw std::invalid_argument("Connectivity edge cost must be finite and nonnegative");
  ConnectivityEdge edge;
  edge.id1_ = id_;
  edge.id2_ = neighbor_id;
  edge.cost_ = cost;
  edge.type_ = type;
  edge.path_ = path;

  const auto existing = std::find_if(neighbors_.begin(), neighbors_.end(),
                                     [neighbor_id](const ConnectivityEdge &candidate) {
                                       return candidate.id2_ == neighbor_id;
                                     });
  if (existing == neighbors_.end())
    neighbors_.push_back(std::move(edge));
  else
    *existing = std::move(edge);
}

void ConnectivityNode::removeNeighbor(const int neighbor_id) {
  neighbors_.erase(std::remove_if(neighbors_.begin(), neighbors_.end(),
                                  [neighbor_id](const ConnectivityEdge &edge) {
                                    return edge.id2_ == neighbor_id;
                                  }),
                   neighbors_.end());
}

void ConnectivityGraph::addNode(const ConnectivityNode::Ptr &node) {
  if (!node || !node->pos_.allFinite())
    throw std::invalid_argument("Connectivity graph nodes must be valid and finite");
  nodes_[node->id_] = node;
}

void ConnectivityGraph::removeNode(const int id) {
  const auto node_it = nodes_.find(id);
  if (node_it == nodes_.end())
    return;
  for (auto &[other_id, node] : nodes_) {
    if (other_id != id && node)
      node->removeNeighbor(id);
  }
  nodes_.erase(node_it);
}

void ConnectivityGraph::clearNodes() { nodes_.clear(); }

ConnectivityNode::Ptr ConnectivityGraph::getNode(const int id) const {
  const auto node_it = nodes_.find(id);
  return node_it == nodes_.end() ? nullptr : node_it->second;
}

void ConnectivityGraph::getNodeNum(int &number) const { number = getNodeNum(); }

int ConnectivityGraph::getNodeNum() const { return static_cast<int>(nodes_.size()); }

void ConnectivityGraph::getNodePositions(std::vector<Position> &positions) const {
  positions.clear();
  positions.reserve(nodes_.size());
  for (const auto &[id, node] : nodes_) {
    (void)id;
    positions.push_back(node->pos_);
  }
}

void ConnectivityGraph::getNodePositionsWithIDs(std::vector<Position> &positions,
                                                std::vector<int> &ids) const {
  positions.clear();
  ids.clear();
  positions.reserve(nodes_.size());
  ids.reserve(nodes_.size());
  for (const auto &[id, node] : nodes_) {
    positions.push_back(node->pos_);
    ids.push_back(id);
  }
}

bool ConnectivityGraph::edgeIsUsable(const ConnectivityEdge &edge) const {
  return std::isfinite(edge.cost_) && edge.cost_ >= 0.0 && edge.cost_ < kBlockedEdgeCost &&
         nodes_.find(edge.id2_) != nodes_.end();
}

double ConnectivityGraph::pathCost(const std::vector<int> &path) const {
  double cost = 0.0;
  for (size_t index = 1; index < path.size(); ++index) {
    const auto source = nodes_.find(path[index - 1]);
    if (source == nodes_.end())
      return kUnreachableCost;
    const auto edge = std::find_if(source->second->neighbors_.begin(),
                                   source->second->neighbors_.end(),
                                   [goal = path[index]](const ConnectivityEdge &candidate) {
                                     return candidate.id2_ == goal;
                                   });
    if (edge == source->second->neighbors_.end() || !edgeIsUsable(*edge))
      return kUnreachableCost;
    cost += edge->cost_;
  }
  return cost;
}

double ConnectivityGraph::searchConnectivityGraphBFS(const int start_id, const int goal_id,
                                                     std::vector<int> &path) const {
  path.clear();
  if (start_id == goal_id) {
    if (nodes_.find(start_id) == nodes_.end())
      return kUnreachableCost;
    path.push_back(start_id);
    return 0.0;
  }
  if (nodes_.find(start_id) == nodes_.end() || nodes_.find(goal_id) == nodes_.end())
    return kUnreachableCost;

  std::queue<int> frontier;
  std::unordered_set<int> visited{start_id};
  std::unordered_map<int, int> parent{{start_id, -1}};
  frontier.push(start_id);

  while (!frontier.empty() && visited.find(goal_id) == visited.end()) {
    const int current = frontier.front();
    frontier.pop();
    for (const ConnectivityEdge &edge : nodes_.at(current)->neighbors_) {
      if (!edgeIsUsable(edge) || visited.find(edge.id2_) != visited.end())
        continue;
      visited.insert(edge.id2_);
      parent[edge.id2_] = current;
      frontier.push(edge.id2_);
    }
  }
  if (visited.find(goal_id) == visited.end())
    return kUnreachableCost;

  for (int current = goal_id; current != -1; current = parent.at(current))
    path.push_back(current);
  std::reverse(path.begin(), path.end());
  return pathCost(path);
}

double ConnectivityGraph::searchConnectivityGraphAStar(const int start_id, const int goal_id,
                                                       std::vector<int> &path) const {
  path.clear();
  if (start_id == goal_id)
    return searchConnectivityGraphBFS(start_id, goal_id, path);
  if (nodes_.find(start_id) == nodes_.end() || nodes_.find(goal_id) == nodes_.end())
    return kUnreachableCost;

  using QueueEntry = std::pair<double, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> frontier;
  std::unordered_map<int, double> distance;
  std::unordered_map<int, int> parent{{start_id, -1}};
  for (const auto &[id, node] : nodes_) {
    (void)node;
    distance[id] = std::numeric_limits<double>::infinity();
  }
  distance[start_id] = 0.0;
  frontier.emplace(0.0, start_id);

  while (!frontier.empty()) {
    const auto [current_cost, current] = frontier.top();
    frontier.pop();
    if (current_cost > distance[current])
      continue;
    if (current == goal_id)
      break;
    for (const ConnectivityEdge &edge : nodes_.at(current)->neighbors_) {
      if (!edgeIsUsable(edge))
        continue;
      const double candidate = current_cost + edge.cost_;
      if (candidate < distance[edge.id2_]) {
        distance[edge.id2_] = candidate;
        parent[edge.id2_] = current;
        frontier.emplace(candidate, edge.id2_);
      }
    }
  }
  if (!std::isfinite(distance[goal_id]))
    return kUnreachableCost;

  for (int current = goal_id; current != -1; current = parent.at(current))
    path.push_back(current);
  std::reverse(path.begin(), path.end());
  return distance[goal_id];
}

void ConnectivityGraph::getFullConnectivityGraph(
    std::vector<std::pair<Position, Position>> &edges,
    std::vector<ConnectivityEdge::TYPE> &edge_types, std::vector<double> &edge_costs) const {
  edges.clear();
  edge_types.clear();
  edge_costs.clear();
  std::set<std::pair<int, int>> visited;
  for (const auto &[id, node] : nodes_) {
    for (const ConnectivityEdge &edge : node->neighbors_) {
      if (!edgeIsUsable(edge))
        continue;
      const auto key = std::minmax(id, edge.id2_);
      if (!visited.emplace(key.first, key.second).second)
        continue;
      edges.emplace_back(node->pos_, nodes_.at(edge.id2_)->pos_);
      edge_types.push_back(edge.type_);
      edge_costs.push_back(edge.cost_);
    }
  }
}

void ConnectivityGraph::getFullConnectivityGraphPath(
    std::vector<std::vector<Position>> &paths,
    std::vector<ConnectivityEdge::TYPE> &path_types) const {
  paths.clear();
  path_types.clear();
  std::set<std::pair<int, int>> visited;
  for (const auto &[id, node] : nodes_) {
    for (const ConnectivityEdge &edge : node->neighbors_) {
      if (!edgeIsUsable(edge))
        continue;
      const auto key = std::minmax(id, edge.id2_);
      if (!visited.emplace(key.first, key.second).second)
        continue;
      if (edge.path_.empty())
        paths.push_back({node->pos_, nodes_.at(edge.id2_)->pos_});
      else
        paths.push_back(edge.path_);
      path_types.push_back(edge.type_);
    }
  }
}

void ConnectivityGraph::findDisconnectedNodes(std::set<int> &disconnected_nodes) const {
  disconnected_nodes.clear();
  std::unordered_set<int> visited;
  for (const auto &[start_id, start_node] : nodes_) {
    if (visited.find(start_id) != visited.end())
      continue;
    std::vector<int> component;
    bool all_unknown = true;
    std::stack<int> pending;
    pending.push(start_id);
    visited.insert(start_id);
    while (!pending.empty()) {
      const int current = pending.top();
      pending.pop();
      component.push_back(current);
      all_unknown = all_unknown && nodes_.at(current)->type_ == ConnectivityNode::TYPE::UNKNOWN;
      for (const ConnectivityEdge &edge : nodes_.at(current)->neighbors_) {
        if (!edgeIsUsable(edge) || visited.find(edge.id2_) != visited.end())
          continue;
        visited.insert(edge.id2_);
        pending.push(edge.id2_);
      }
    }
    if (all_unknown)
      disconnected_nodes.insert(component.begin(), component.end());
  }
}

bool ConnectivityGraph::saveConnectivityGraph(const std::string &file_path) const {
  std::ofstream file(file_path);
  if (!file.is_open())
    return false;
  file << std::setprecision(std::numeric_limits<double>::max_digits10);

  std::vector<int> ids;
  ids.reserve(nodes_.size());
  for (const auto &[id, node] : nodes_) {
    (void)node;
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  for (const int id : ids) {
    const auto &node = nodes_.at(id);
    file << node->id_ << ' ' << node->pos_.x() << ' ' << node->pos_.y() << ' '
         << node->pos_.z() << ' ' << static_cast<int>(node->type_) << '\n';
    std::vector<ConnectivityEdge> neighbors = node->neighbors_;
    std::sort(neighbors.begin(), neighbors.end(), [](const auto &left, const auto &right) {
      return left.id2_ < right.id2_;
    });
    for (const ConnectivityEdge &edge : neighbors)
      file << edge.id2_ << ' ' << edge.cost_ << ' ' << static_cast<int>(edge.type_) << '\n';
    file << '\n';
  }
  return file.good();
}

bool ConnectivityGraph::loadConnectivityGraph(const std::string &file_path) {
  std::ifstream file(file_path);
  if (!file.is_open())
    return false;

  ConnectivityGraph parsed;
  std::string line;
  while (std::getline(file, line)) {
    if (blankLine(line))
      continue;
    std::istringstream node_stream(line);
    int id = -1;
    int type_value = -1;
    Position position = Position::Zero();
    if (!(node_stream >> id >> position.x() >> position.y() >> position.z() >> type_value) ||
        !position.allFinite() || type_value < static_cast<int>(ConnectivityNode::TYPE::UNKNOWN) ||
        type_value > static_cast<int>(ConnectivityNode::TYPE::FREE))
      return false;
    auto node = std::make_shared<ConnectivityNode>(
        id, position, static_cast<ConnectivityNode::TYPE>(type_value));

    while (std::getline(file, line) && !blankLine(line)) {
      std::istringstream edge_stream(line);
      int neighbor_id = -1;
      int edge_type_value = -1;
      double cost = 0.0;
      if (!(edge_stream >> neighbor_id >> cost >> edge_type_value) || !std::isfinite(cost) ||
          cost < 0.0 || edge_type_value < static_cast<int>(ConnectivityEdge::TYPE::UNKNOWN) ||
          edge_type_value > static_cast<int>(ConnectivityEdge::TYPE::PORTAL))
        return false;
      node->addNeighbor(neighbor_id, cost,
                        static_cast<ConnectivityEdge::TYPE>(edge_type_value));
    }
    parsed.addNode(node);
  }

  for (const auto &[id, node] : parsed.nodes_) {
    (void)id;
    for (const ConnectivityEdge &edge : node->neighbors_) {
      if (parsed.nodes_.find(edge.id2_) == parsed.nodes_.end())
        return false;
    }
  }
  nodes_ = std::move(parsed.nodes_);
  return true;
}

} // namespace fast_planner
