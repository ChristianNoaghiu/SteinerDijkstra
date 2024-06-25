#include <stdexcept>
#include <limits>
#include "steinergraph.h"

const int SteinerGraph::infinite_weight = std::numeric_limits<int>::max();
const int SteinerGraph::infinite_distance = std::numeric_limits<int>::max();

void SteinerGraph::add_nodes(const NodeId num_new_nodes)
{
   _nodes.resize(num_nodes() + num_new_nodes);
}

void SteinerGraph::make_terminal(const NodeId new_terminal)
{
   if (new_terminal >= num_nodes() or new_terminal < 0)
   {
      throw std::runtime_error("Terminalstatus of Node cannot be changed due to undefined node");
   }
   _nodes[new_terminal].set_terminal();

   // insert new_terminal into _terminal_vector if not already present
   if (std::find(_terminals.begin(), _terminals.end(), new_terminal) == _terminals.end())
   {
      _terminals.push_back(new_terminal);
   }
}

void SteinerGraph::set_predecessor(const NodeId node_id, const std::optional<NodeId> predecessor)
{
   if (node_id >= num_nodes() or node_id < 0)
   {
      throw std::runtime_error("Undefined node");
   }

   if (predecessor.has_value())
   {
      if (predecessor.value() >= num_nodes() || predecessor.value() < 0)
      {
         throw std::runtime_error("Undefined predecessor");
      }
   }

   _nodes.at(node_id).set_predecessor(predecessor);
}

void SteinerGraph::Node::set_terminal()
{
   _terminal = true;
}

void SteinerGraph::Node::set_predecessor(const std::optional<NodeId> predecessor)
{
   _predecessor = predecessor;
}

const std::optional<SteinerGraph::NodeId> &SteinerGraph::Node::get_predecessor() const
{
   return _predecessor;
}

SteinerGraph::Neighbor::Neighbor(const SteinerGraph::NodeId n, const int w) : _id(n), _edge_weight(w) {}

SteinerGraph::SteinerGraph(const NodeId num) : _nodes(num) {}

// returns a graph with the same nodes and terminals,
// but without edges
SteinerGraph SteinerGraph::clear_edges() const
{
   SteinerGraph result_graph(num_nodes());

   for (NodeId terminal : _terminals)
   {
      result_graph.make_terminal(terminal);
   }

   return result_graph;
}

void SteinerGraph::add_edge(const NodeId tail, const NodeId head, const int weight)
{
   if (tail >= num_nodes() or tail < 0 or head >= num_nodes() or head < 0)
   {
      throw std::runtime_error("Edge cannot be added due to undefined endpoint.");
   }
   _nodes[tail].add_neighbor(head, weight);
   _nodes[head].add_neighbor(tail, weight);
}

void SteinerGraph::Node::add_neighbor(const SteinerGraph::NodeId nodeid, const int weight)
{
   _neighbors.push_back(SteinerGraph::Neighbor(nodeid, weight));
}

const std::vector<SteinerGraph::Neighbor> &SteinerGraph::Node::adjacent_nodes() const
{
   return _neighbors;
}

SteinerGraph::NodeId SteinerGraph::num_nodes() const
{
   return _nodes.size();
}

SteinerGraph::NodeId SteinerGraph::num_terminals() const
{
   return _terminals.size();
}

const SteinerGraph::Node &SteinerGraph::get_node(const NodeId node) const
{
   if (node < 0 or node >= static_cast<int>(_nodes.size()))
   {
      throw std::runtime_error("Invalid nodeid in Graph::get_node.");
   }
   return _nodes[node];
}

SteinerGraph::NodeId SteinerGraph::Neighbor::id() const
{
   return _id;
}

bool SteinerGraph::Node::is_terminal() const
{
   return _terminal;
}

int SteinerGraph::Neighbor::edge_weight() const
{
   return _edge_weight;
}

int SteinerGraph::edge_weight_sum() const
{
   int result = 0;

   for (auto nodeid = 0; nodeid < num_nodes(); ++nodeid)
   {
      for (auto neighbor : get_node(nodeid).adjacent_nodes())
      {
         if (neighbor.id() > nodeid) // ensures that edge are counted only once
         {
            result += neighbor.edge_weight();
         }
      }
   }

   return result;
}

void SteinerGraph::print() const
{
   std::cout << "SteinerGraph ";

   int terminalcounter = 0;
   std::vector<int> terminalliste;
   for (int nodeid = 0; nodeid < num_nodes(); ++nodeid) // here we count the Terminals and save them in a vector for their following output
   {
      if (get_node(nodeid).is_terminal())
      {
         terminalcounter++;
         terminalliste.push_back(nodeid);
      }
   }

   std::cout << "with " << num_nodes() << " vertices, numbered 0,...,"
             << num_nodes() - 1 << " and " << terminalcounter << " terminals." << std::endl;
   std::cout << "The terminals are the following nodes: ";
   for (unsigned int i = 0; i < terminalliste.size(); i++)
   {
      std::cout << terminalliste.at(i) << ", ";
   }
   std::cout << std::endl
             << std::endl;

   for (auto nodeid = 0; nodeid < num_nodes(); ++nodeid)
   {
      std::cout << "The following edges are incident to vertex " << nodeid << ":\n";
      for (auto neighbor : get_node(nodeid).adjacent_nodes())
      {
         std::cout << nodeid << " - " << neighbor.id()
                   << " weight = " << neighbor.edge_weight() << "\n";
      }
   }

   std::cout << "\nSum of edge weights: " << edge_weight_sum() << "\n";
}

void SteinerGraph::check_valid_node(const NodeId node) const
{
   if (node < 0 || node >= num_nodes())
   {
      throw std::runtime_error("Invalid NodeId.");
   }
}

void SteinerGraph::check_valid_terminal(const TerminalId terminal) const
{
   if (terminal < 0 || terminal >= num_terminals())
   {
      throw std::runtime_error("Invalid TerminalId.");
   }
   if (terminal >= 64)
   {
      throw std::runtime_error("TerminalId exceeds the size of TerminalSubset.");
   }
}