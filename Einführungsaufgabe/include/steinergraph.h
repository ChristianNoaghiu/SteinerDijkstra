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

    void set_terminal();
    bool is_terminal() const;

  private:
    std::vector<Neighbor> _neighbors;
    bool _terminal = false;
  };

  SteinerGraph(NodeId num_nodes);
  SteinerGraph(char const *filename);

  void add_nodes(NodeId num_new_nodes);
  void add_edge(NodeId tail, NodeId head, double weight = 1.0);
  void add_terminal(NodeId new_terminal);

  void dijkstra(const NodeId start_node, std::vector<int> &distances, std::vector<NodeId> &predecessors) const;
  void metric_closure(std::vector<std::vector<int>> &distance_matrix, std::vector<std::vector<NodeId>> &predecessor_matrix) const;

  NodeId num_nodes() const;
  const Node &get_node(NodeId) const;
  void print() const;

  static const NodeId invalid_node;
  static const double infinite_weight;
  static const int infinite_distance;

private:
  std::vector<Node> _nodes;
};
