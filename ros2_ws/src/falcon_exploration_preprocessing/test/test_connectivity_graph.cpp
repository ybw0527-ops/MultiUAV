#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include "exploration_preprocessing/connectivity_graph.h"

namespace fast_planner {
namespace {

ConnectivityNode::Ptr addNode(ConnectivityGraph &graph, const int id,
                              const ConnectivityNode::TYPE type = ConnectivityNode::TYPE::FREE) {
  auto node = std::make_shared<ConnectivityNode>(id, Position(static_cast<double>(id), 0.0, 0.0),
                                                 type);
  graph.addNode(node);
  return node;
}

void connect(const ConnectivityNode::Ptr &left, const ConnectivityNode::Ptr &right,
             const double cost, const ConnectivityEdge::TYPE type = ConnectivityEdge::TYPE::FREE) {
  left->addNeighbor(right->id_, cost, type);
  right->addNeighbor(left->id_, cost, type);
}

TEST(ConnectivityGraphTest, FindsHopAndMinimumCostPaths) {
  ConnectivityGraph graph;
  const auto one = addNode(graph, 1);
  const auto two = addNode(graph, 2);
  const auto three = addNode(graph, 3);
  const auto four = addNode(graph, 4);
  connect(one, two, 2.0);
  connect(two, three, 3.0);
  connect(one, four, 1.0);
  connect(four, three, 10.0);

  std::vector<int> path;
  EXPECT_DOUBLE_EQ(graph.searchConnectivityGraphBFS(1, 3, path), 5.0);
  EXPECT_EQ(path, (std::vector<int>{1, 2, 3}));

  path.clear();
  EXPECT_DOUBLE_EQ(graph.searchConnectivityGraphAStar(1, 3, path), 5.0);
  EXPECT_EQ(path, (std::vector<int>{1, 2, 3}));

  path.clear();
  EXPECT_DOUBLE_EQ(graph.searchConnectivityGraphBFS(1, 1, path), 0.0);
  EXPECT_EQ(path, (std::vector<int>{1}));
  EXPECT_DOUBLE_EQ(graph.searchConnectivityGraphBFS(1, 99, path),
                   ConnectivityGraph::kUnreachableCost);
  EXPECT_TRUE(path.empty());
}

TEST(ConnectivityGraphTest, RemovesEdgesAndClassifiesUnknownComponents) {
  ConnectivityGraph graph;
  const auto unknown_one = addNode(graph, 10, ConnectivityNode::TYPE::UNKNOWN);
  const auto unknown_two = addNode(graph, 11, ConnectivityNode::TYPE::UNKNOWN);
  const auto free_node = addNode(graph, 20, ConnectivityNode::TYPE::FREE);
  connect(unknown_one, unknown_two, 1.0, ConnectivityEdge::TYPE::UNKNOWN);

  std::set<int> disconnected;
  graph.findDisconnectedNodes(disconnected);
  EXPECT_EQ(disconnected, (std::set<int>{10, 11}));

  connect(unknown_two, free_node, 2.0, ConnectivityEdge::TYPE::PORTAL);
  graph.findDisconnectedNodes(disconnected);
  EXPECT_TRUE(disconnected.empty());

  graph.removeNode(11);
  EXPECT_EQ(graph.getNodeNum(), 2);
  ASSERT_TRUE(graph.getNode(10));
  EXPECT_TRUE(graph.getNode(10)->neighbors_.empty());
  EXPECT_EQ(graph.getNode(11), nullptr);
}

TEST(ConnectivityGraphTest, SavesAndLoadsCostsWithoutChangingUnits) {
  ConnectivityGraph original;
  const auto one = addNode(original, 1);
  const auto two = addNode(original, 2, ConnectivityNode::TYPE::UNKNOWN);
  connect(one, two, 2.75, ConnectivityEdge::TYPE::PORTAL);

  const std::filesystem::path file_path =
      std::filesystem::temp_directory_path() / "falcon_connectivity_graph_test.txt";
  std::filesystem::remove(file_path);
  ASSERT_TRUE(original.saveConnectivityGraph(file_path.string()));

  ConnectivityGraph loaded;
  ASSERT_TRUE(loaded.loadConnectivityGraph(file_path.string()));
  EXPECT_EQ(loaded.getNodeNum(), 2);
  std::vector<int> path;
  EXPECT_DOUBLE_EQ(loaded.searchConnectivityGraphAStar(1, 2, path), 2.75);
  EXPECT_EQ(path, (std::vector<int>{1, 2}));
  ASSERT_TRUE(loaded.getNode(2));
  EXPECT_EQ(loaded.getNode(2)->type_, ConnectivityNode::TYPE::UNKNOWN);

  {
    std::ofstream malformed(file_path);
    malformed << "1 0 0 invalid\n";
  }
  EXPECT_FALSE(loaded.loadConnectivityGraph(file_path.string()));
  EXPECT_EQ(loaded.getNodeNum(), 2);
  std::filesystem::remove(file_path);
}

} // namespace
} // namespace fast_planner
