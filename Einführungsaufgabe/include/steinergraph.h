#pragma once

#include <iostream>
#include <vector>
#include <set>

class SteinerGraph
{
public:
  using NodeId = int; // vertices are numbered 0,...,num_nodes()-1

  class Neighbor
  {
  public:
    Neighbor(SteinerGraph::NodeId n, int w);
    int edge_weight() const;
    SteinerGraph::NodeId id() const;

  private:
    SteinerGraph::NodeId _id;
    int _edge_weight;
  };

  class Node
  {
  public:
    void add_neighbor(SteinerGraph::NodeId nodeid, int weight);
    const std::vector<Neighbor> &adjacent_nodes() const;

    void set_terminal();
    bool is_terminal() const;

  private:
    std::vector<Neighbor> _neighbors;
    bool _terminal = false;
  };

  SteinerGraph(NodeId num_nodes);
  SteinerGraph(char const *filename);

  void add_nodes(NodeId num_new_nodes);
  void add_edge(NodeId tail, NodeId head, int weight = 1);
  void make_terminal(NodeId new_terminal);

  void dijkstra(
      const NodeId start_node,
      std::vector<int> &distances,
      std::vector<NodeId> &predecessors,
      std::vector<int> &predecessor_weights)
      const;

  void metric_closure(
      std::vector<std::vector<int>> &distance_matrix,
      std::vector<std::vector<NodeId>> &predecessor_matrix,
      std::vector<std::vector<int>> &predecessor_distance_matrix)
      const;

  NodeId find_terminal_node() const;

  SteinerGraph steiner_tree_mst_approximation() const;

  NodeId num_nodes() const;
  const Node &get_node(NodeId) const;
  void print() const;

  static const NodeId invalid_node;
  static const int infinite_weight;
  static const int infinite_distance;

private:
  void terminal_rooted_mst(
      const std::vector<std::vector<int>> &metric_closure_distance_matrix,
      std::vector<NodeId> &predecessors)
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
  std::set<NodeId> _terminals;
};
