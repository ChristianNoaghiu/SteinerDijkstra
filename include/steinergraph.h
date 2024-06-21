#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <set>
#include <bitset>
#include <unordered_map>
#include <unordered_set>

class SteinerGraph
{
public:
  using NodeId = int; // vertices are numbered 0,...,num_nodes()-1
  using TerminalId = int;
  using TerminalSubset = std::bitset<64>;

  class Neighbor
  {
  public:
    Neighbor(const SteinerGraph::NodeId n, const int w);
    int edge_weight() const;
    SteinerGraph::NodeId id() const;

  private:
    SteinerGraph::NodeId _id;
    int _edge_weight;
  };

  class Node
  {
  public:
    void add_neighbor(const SteinerGraph::NodeId nodeid, const int weight);
    const std::vector<Neighbor> &adjacent_nodes() const;

    void set_terminal();
    void set_predecessor(const std::optional<NodeId> predecessor);
    const std::optional<NodeId> &get_predecessor() const;
    bool is_terminal() const;

  private:
    std::vector<Neighbor> _neighbors;
    std::optional<NodeId> _predecessor = {}; // for MST information
    bool _terminal = false;
  };

  SteinerGraph(const NodeId num_nodes);
  SteinerGraph(char const *filename);

  SteinerGraph clear_edges() const;

  void add_nodes(const NodeId num_new_nodes);
  void add_edge(const NodeId tail, const NodeId head, const int weight = 1);
  void make_terminal(const NodeId new_terminal);
  void set_predecessor(const NodeId node_id, const std::optional<NodeId> predecessor);

  struct DijkstraStruct
  {
    std::vector<int> distances;
    std::vector<std::optional<NodeId>> predecessors;
    std::vector<int> predecessor_weights;
  };
  DijkstraStruct dijkstra(const NodeId start_node) const;

  struct MetricClosureStruct
  {
    std::vector<std::vector<int>> distance_matrix;
    std::vector<std::vector<std::optional<NodeId>>> predecessor_matrix;
    std::vector<std::vector<int>> predecessor_weight_matrix;
  };
  MetricClosureStruct metric_closure() const;
  SteinerGraph metric_closure_graph(
      const std::vector<std::vector<int>> metric_closure_distance_matrix)
      const;

  std::optional<NodeId> find_terminal_node() const;

  SteinerGraph steiner_tree_mst_approximation() const;

  SteinerGraph subgraph_mst(
      const std::function<bool(const NodeId node)> is_in_subgraph)
      const;
  SteinerGraph subgraph_mst(
      const std::function<bool(const NodeId node)> is_in_subgraph,
      const NodeId start_node)
      const;
  SteinerGraph component_mst(const NodeId start_node) const;

  NodeId num_nodes() const;
  TerminalId num_terminals() const;
  const Node &get_node(const NodeId node) const;
  int edge_weight_sum() const;
  void print() const;

  static const int infinite_weight;
  static const int infinite_distance;

  /** @todo remove this */
  void test_one_tree_bound();

private:
  void check_valid_node(const NodeId node) const;
  void check_valid_terminal(const TerminalId node) const;

  void check_connected_metric_closure(
      const std::vector<std::vector<int>> &metric_closure_distance_matrix)
      const;

  void add_path_to_steiner_tree_mst_approximation(
      const NodeId &start_node,
      const std::vector<std::vector<std::optional<NodeId>>> &metric_closure_predecessor_matrix,
      const std::vector<std::vector<int>> &metric_closure_predecessor_weight_matrix,
      const SteinerGraph &mst_graph,
      std::vector<bool> &visited,
      SteinerGraph &result_graph)
      const;

  std::vector<Node> _nodes;
  /** @todo replace this */
  std::unordered_set<NodeId> _terminals;
  std::vector<NodeId> _terminals_vector;

  // for queues in Dijkstra's and Prim's algorithms
  using NodeDistancePair = std::pair<SteinerGraph::NodeId, int>;
  static std::function<bool(
      const NodeDistancePair,
      const NodeDistancePair)>
  node_distance_pair_compare();

  const std::function<bool(const SteinerGraph::NodeId)> is_in_graph() const;
  const std::function<bool(const SteinerGraph::NodeId)> is_in_set(const std::set<SteinerGraph::NodeId> &node_set) const;
  const std::function<bool(const SteinerGraph::NodeId)> is_in_terminal_subset(const TerminalSubset &terminal_subset) const;

  /** @todo outsource this to separate algorithm class */
  struct PairHash
  {
  public:
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U> &x) const
    {
      return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
    }
  };

  using BoundKey = std::pair<NodeId, TerminalSubset>;
  using BoundKeyToDoubleMap = std::unordered_map<BoundKey, double, PairHash>;

  std::optional<TerminalId> _computed_one_tree_bound_root_terminal;
  BoundKeyToDoubleMap _computed_one_tree_bounds;
  double get_or_compute_one_tree_bound(
      const NodeId node,
      const TerminalSubset &terminal_subset,
      const TerminalId r0);

  static int terminal_subset_size(const TerminalSubset &terminal_subset);
};
