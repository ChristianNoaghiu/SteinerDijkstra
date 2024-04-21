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
    Neighbor(SteinerGraph::NodeId n, double w, bool t);
    double edge_weight() const;
    SteinerGraph::NodeId id() const;
    bool isTerminal() const;
    void setTerminal();

  private:
    SteinerGraph::NodeId _id;
    double _edge_weight;
    bool _terminal;
  };

  class Node
  {
  public:
    void add_neighbor(SteinerGraph::NodeId nodeid, double weight, bool terminal = false);
    const std::vector<Neighbor> &adjacent_nodes() const;
    void find_to_setTerminal(NodeId);

  private:
    std::vector<Neighbor> _neighbors;
  };

  SteinerGraph(NodeId num_nodes);
  SteinerGraph(char const *filename);

  void add_nodes(NodeId num_new_nodes);
  void add_edge(NodeId tail, NodeId head, double weight = 1.0);
  void add_terminal(NodeId new_terminal);

  NodeId num_nodes() const;
  const Node &get_node(NodeId) const;
  void print() const;

  static const NodeId invalid_node;
  static const double infinite_weight;

private:
  std::vector<Node> _nodes;
  std::vector<NodeId> _terminals;
};
