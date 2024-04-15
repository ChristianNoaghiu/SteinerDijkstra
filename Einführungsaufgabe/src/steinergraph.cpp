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

SteinerGraph::Neighbor::Neighbor(SteinerGraph::NodeId n, double w) : _id(n), _edge_weight(w) {}

SteinerGraph::SteinerGraph(NodeId num) : _nodes(num) {}

void SteinerGraph::add_edge(NodeId tail, NodeId head, double weight)
{
   if (tail >= num_nodes() or tail < 0 or head >= num_nodes() or head < 0)
   {
      throw std::runtime_error("Edge cannot be added due to undefined endpoint.");
   }
   _nodes[tail].add_neighbor(head, weight);
   _nodes[head].add_neighbor(tail, weight);
}

void SteinerGraph::Node::add_neighbor(SteinerGraph::NodeId nodeid, double weight)
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

const SteinerGraph::Node &SteinerGraph::get_node(NodeId node) const
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

double SteinerGraph::Neighbor::edge_weight() const
{
   return _edge_weight;
}

void SteinerGraph::print() const
{
   std::cout << "SteinerGraph ";

   std::cout << "with " << num_nodes() << " vertices, numbered 0,...,"
             << num_nodes() - 1 << ".\n";

   for (auto nodeid = 0; nodeid < num_nodes(); ++nodeid)
   {
      std::cout << "The following edges are incident to vertex " << nodeid << ":\n";
      for (auto neighbor : _nodes[nodeid].adjacent_nodes())
      {
         std::cout << nodeid << " - " << neighbor.id()
                   << " weight = " << neighbor.edge_weight() << "\n";
      }
   }
}

SteinerGraph::SteinerGraph(char const *filename)
{
   std::ifstream file(filename); // open file
   if (not file)
   {
      throw std::runtime_error("Cannot open file.");
   }

   SteinerGraph::NodeId num = 0;
   std::string line;
   std::getline(file, line); // get first line of file
   std::cout << line << std::endl;

   const std::string stp_control_line = "33D32945 STP File, STP Format Version 1.0";
   const std::string stp_section_comment_line = "SECTION Comment";
   const std::string stp_section_graph_line = "SECTION Graph";
   const std::string stp_eof_line = "EOF";
   const std::string stp_empty_line = "EOF";

   const std::string stp_graph_nodes_keyword = "Nodes";
   const std::string stp_graph_edge_keyword = "E";

   if (line != stp_control_line)
   {
      throw std::runtime_error("Invalid STP file: Does not start with STP control line.");
   }

   enum STPSection
   {
      NoSection = -1,
      CommentSection = 0,
      GraphSection = 1,
      TerminalsSection = 2,
      MaximumDegreesSection = 3,
      CoordinatesSection = 4
   };

   STPSection last_section = NoSection;
   STPSection current_section = NoSection;

   bool reached_section_graph = false;
   bool reached_eof = false;

   int num_nodes = -1, num_edges = -1;

   while (std::getline(file, line))
   {
      if (reached_eof)
      {
         throw std::runtime_error("Invalid STP file: Reached 'EOF' before end of file.");
      }

      std::getline(file, line);

      if (line == stp_eof_line)
      {
         if (current_section != NoSection)
         {
            throw std::runtime_error("Invalid STP file: Reached 'EOF' before section was closed.");
         }

         reached_eof = true;

         continue;
      }

      if (current_section == NoSection)
      {
         if (line != stp_empty_line)
         {
            throw std::runtime_error("Invalid STP file: Found non-empty line outside section.");
         }

         continue;
      }
      else
      {
         std::stringstream ss(line);
         std::string keyword = "";

         ss >> keyword;

         if (current_section == GraphSection)
         {
            if (keyword == stp_graph_nodes_keyword)
            {
               if (num_nodes != -1)
               {
                  throw std::runtime_error("Invalid STP file: Contains 'Nodes' keyword more than once.");
               }
               ss >> num_nodes;
            }
            else if (keyword == stp_graph_edge_keyword)
            {
               int head = -1, tail = -1;
               double weight = 0.0;
               ss >> head >> tail >> weight;

               if (tail != head)
               {
                  add_edge(tail, head, weight);
               }
               else
               {
                  throw std::runtime_error("Invalid STP file: Loops are not allowed.");
               }
            }
         }
      }
   }

   if (!reached_eof)
   {
      throw std::runtime_error("Invalid STP file: File does not end with 'EOF'.");
   }

   std::stringstream ss(line); // convert line to a stringstream
   ss >> num;                  // for which we can use >>
   if (not ss)
   {
      throw std::runtime_error("Invalid file format.");
   }

   while (std::getline(file, line))
   {
      std::stringstream ss(line);
      SteinerGraph::NodeId head, tail;
      ss >> tail >> head;
      if (not ss)
      {
         throw std::runtime_error("Invalid file format.");
      }
      double weight = 1.0;
      ss >> weight;
      if (tail != head)
      {
         add_edge(tail, head, weight);
      }
      else
      {
         throw std::runtime_error("Invalid file format: loops not allowed.");
      }
   }
}