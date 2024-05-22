#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <set>

class SteinerGraph
{
public:
  using NodeId = int; // vertices are numbered 0,...,num_nodes()-1

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
  const Node &get_node(const NodeId node) const;
  int edge_weight_sum() const;
  void print() const;

  static const int infinite_weight;
  static const int infinite_distance;

private:
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
  std::set<NodeId> _terminals;

  // lambda returning
  std::function<bool(const SteinerGraph::NodeId)> is_in_graph();

  // lambda returning whether a node is contained in node_set
  std::function<bool(const SteinerGraph::NodeId)> is_in_set(std::set<SteinerGraph::NodeId> node_set);
};
