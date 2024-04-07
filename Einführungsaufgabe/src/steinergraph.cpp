#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include "steinergraph.h"

const SteinerGraph::NodeId SteinerGraph::invalid_node = -1;
const double SteinerGraph::infinite_weight = std::numeric_limits<double>::max();


void SteinerGraph::add_nodes(NodeId num_new_nodes)
{
   _nodes.resize(num_nodes() + num_new_nodes);
}

SteinerGraph::Neighbor::Neighbor(SteinerGraph::NodeId n, double w): _id(n), _edge_weight(w) {}

SteinerGraph::SteinerGraph(NodeId num, DirType dtype): dirtype(dtype), _nodes(num) {}

void SteinerGraph::add_edge(NodeId tail, NodeId head, double weight)
{
   if (tail >= num_nodes() or tail < 0 or head >= num_nodes() or head < 0) {
       throw std::runtime_error("Edge cannot be added due to undefined endpoint.");
   }
   _nodes[tail].add_neighbor(head, weight);
   if (dirtype == SteinerGraph::undirected) {
        _nodes[head].add_neighbor(tail, weight);
   }
}

void SteinerGraph::Node::add_neighbor(SteinerGraph::NodeId nodeid, double weight)
{
   _neighbors.push_back(SteinerGraph::Neighbor(nodeid, weight));
}

const std::vector<SteinerGraph::Neighbor> & SteinerGraph::Node::adjacent_nodes() const
{
   return _neighbors;
}

SteinerGraph::NodeId SteinerGraph::num_nodes() const
{
   return _nodes.size();
}

const SteinerGraph::Node & SteinerGraph::get_node(NodeId node) const
{
   if (node < 0 or node >= static_cast<int>(_nodes.size())) {
        throw std::runtime_error("Invalid nodeid in Graph::get_node.");
   }
   return _nodes[node];
}

SteinerGraph::NodeId SteinerGraph::Neighbor::id() const
{
   return _id;
}

double SteinerGraph::Neighbor::edge_weight() const
{
   return _edge_weight;
}

void SteinerGraph::print() const
{
   if (dirtype == SteinerGraph::directed) {
        std::cout << "Digraph ";
   } else {
        std::cout << "Undirected graph ";
   }
   std::cout << "with " << num_nodes() << " vertices, numbered 0,...,"
              << num_nodes() - 1 << ".\n";

   for (auto nodeid = 0; nodeid < num_nodes(); ++nodeid) {
        std::cout << "The following edges are ";
        if (dirtype == SteinerGraph::directed) {
            std::cout << "leaving";
        } else {
            std::cout << "incident to";
        }
        std::cout << " vertex " << nodeid << ":\n";
        for (auto neighbor: _nodes[nodeid].adjacent_nodes()) {
            std::cout << nodeid << " - " << neighbor.id()
                      << " weight = " << neighbor.edge_weight() << "\n";
        }
   }
}

SteinerGraph::SteinerGraph(char const * filename, DirType dtype): dirtype(dtype)
{
   std::ifstream file(filename);                             // open file
   if (not file) {
        throw std::runtime_error("Cannot open file.");
   }

   SteinerGraph::NodeId num = 0;
   std::string line;
   std::getline(file, line);                 // get first line of file
   std::stringstream ss(line);               // convert line to a stringstream
   ss >> num;                                // for which we can use >>
   if (not ss) {
        throw std::runtime_error("Invalid file format.");
   }
   add_nodes(num);

   while (std::getline(file, line)) {
        std::stringstream ss(line);
        SteinerGraph::NodeId head, tail;
        ss >> tail >> head;
        if (not ss) {
            throw std::runtime_error("Invalid file format.");
        }
        double weight = 1.0;
        ss >> weight;
        if (tail != head) {
            add_edge(tail, head, weight);
        }
        else {
            throw std::runtime_error("Invalid file format: loops not allowed.");
        }
   }
}