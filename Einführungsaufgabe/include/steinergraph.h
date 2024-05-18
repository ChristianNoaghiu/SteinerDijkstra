#pragma once

#include <iostream>
#include <vector>

class SteinerGraph
{
public:
  using NodeId = int; // vertices are numbered 0,...,num_nodes()-1

  class Neighbor
  {
  public:
    Neighbor(SteinerGraph::NodeId n, double w);
    double edge_weight() const;
    SteinerGraph::NodeId id() const;

  private:
    SteinerGraph::NodeId _id;
    double _edge_weight;
  };

  class Node
  {
  public:
    void add_neighbor(SteinerGraph::NodeId nodeid, double weight);
    const std::vector<Neighbor> &adjacent_nodes() const;

  private:
    std::vector<Neighbor> _neighbors;
  };

  SteinerGraph(NodeId num_nodes);
  SteinerGraph(char const *filename);

  void add_nodes(NodeId num_new_nodes);
  void add_edge(NodeId tail, NodeId head, int weight = 1);
  void make_terminal(NodeId new_terminal);

  struct DijkstraStruct
  {
    std::vector<int> distances;
    std::vector<NodeId> predecessors;
    std::vector<int> predecessor_weights;
  };
  DijkstraStruct dijkstra(const NodeId start_node) const;

  struct MetricClosureStruct
  {
    std::vector<std::vector<int>> distance_matrix;
    std::vector<std::vector<NodeId>> predecessor_matrix;
    std::vector<std::vector<int>> predecessor_weight_matrix;
  };
  MetricClosureStruct metric_closure() const;

  NodeId find_terminal_node() const;

  SteinerGraph steiner_tree_mst_approximation() const;

  SteinerGraph component_mst(const NodeId start_node) const;

  NodeId num_nodes() const;
  const Node &get_node(NodeId) const;
  void print() const;

  static const NodeId invalid_node;
  static const double infinite_weight;

private:
  void check_connected_metric_closure(
      const std::vector<std::vector<int>> &metric_closure_distance_matrix)
      const;

  std::vector<NodeId> terminal_rooted_mst_predecessors(
      const std::vector<std::vector<int>> &metric_closure_distance_matrix)
      const;

  void add_path_to_steiner_tree_mst_approximation(
      const NodeId &start_node,
      const std::vector<std::vector<NodeId>> &metric_closure_predecessor_matrix,
      const std::vector<std::vector<int>> &metric_closure_predecessor_weight_matrix,
      const std::vector<NodeId> &mst_predecessors,
      std::vector<bool> &visited,
      SteinerGraph &result_graph)
      const;

  std::vector<Node> _nodes;
};
