#include <fstream>
#include <sstream>
#include <stdexcept>
#include <limits>
#include "steinergraph.h"

namespace
{
   const std::string stp_control_line = "33D32945 STP File, STP Format Version 1.0";
   const std::string stp_section_comment_line = "SECTION Comment";
   const std::string stp_section_graph_line = "SECTION Graph";
   const std::string stp_section_terminals_line = "SECTION Terminals";
   const std::string stp_section_maximumdegrees_line = "SECTION MaximumDegrees";
   const std::string stp_section_coordinates_line = "SECTION Coordinates";
   const std::string stp_eof_line = "EOF";
   const std::string stp_end_line = "END";
   const std::string stp_empty_line = "EOF"; // Isn't this line supposed to be empty according to the name?

   const std::string stp_graph_nodes_keyword = "Nodes";
   const std::string stp_graph_edges_keyword = "Edges";
   const std::string stp_graph_edge_keyword = "E";

   const std::string stp_terminals_terminals_keyword = "Terminals";
   const std::string stp_terminals_terminal_keyword = "T";

   enum STPSection
   {
      NoSection = -1,
      CommentSection = 0,   // skip through and don't do anything
      GraphSection = 1,     // defines the graph
      TerminalsSection = 2, // saves the terminals in the terminals-section
      MaximumDegreesSection = 3,
      CoordinatesSection = 4
   };

   bool check_for_sections(STPSection &_section, STPSection &_last_section, const std::string &_line)
   {
      if (_line == stp_section_comment_line)
      {
         _section = CommentSection;
         if (_section <= _last_section)
         {
            throw std::runtime_error("Invalid STP file: Sections are not ordered correctly.");
         }
         _last_section = CommentSection;
         return true;
      }
      else if (_line == stp_section_graph_line)
      {
         _section = GraphSection;
         if (_section <= _last_section)
         {
            throw std::runtime_error("Invalid STP file: Sections are not ordered correctly.");
         }
         return true;
      }
      else if (_line == stp_section_terminals_line)
      {
         _section = TerminalsSection;
         if (_section <= _last_section)
         {
            throw std::runtime_error("Invalid STP file: Sections are not ordered correctly.");
         }
         return true;
      }
      else if (_line == stp_section_maximumdegrees_line)
      {
         _section = MaximumDegreesSection;
         if (_section <= _last_section)
         {
            throw std::runtime_error("Invalid STP file: Sections are not ordered correctly.");
         }
         _last_section = MaximumDegreesSection;
         return true;
      }
      else if (_line == stp_section_coordinates_line)
      {
         _section = CoordinatesSection;
         if (_section <= _last_section)
         {
            throw std::runtime_error("Invalid STP file: Sections are not ordered correctly.");
         }
         _last_section = CoordinatesSection;
         return true;
      }
      else
      {
         return false;
      }
   }

   int graph_section(const std::string &_line, int &_num_nodes, int &_num_edges, int _head, int _tail, double &_weight, unsigned int &_edge_counter)
   {
      std::stringstream ss(_line);
      std::string _keyword = "";
      ss >> _keyword;
      if (_keyword == stp_graph_nodes_keyword)
      {
         if (_num_nodes != -1)
         {
            throw std::runtime_error("Invalid STP file: Contains 'Nodes' keyword more than once.");
         }
         ss >> _num_nodes;
         if (_num_nodes < 0)
         {
            throw std::runtime_error("Invalid STP file: The number of nodes cannot be negative.");
         }
         return 0;
      }
      else if (_keyword == stp_graph_edges_keyword)
      {
         if (_num_edges != -1)
         {
            throw std::runtime_error("Invalid STP file: Contains 'Nodes' keyword more than once.");
         }
         ss >> _num_edges;
         if (_num_edges < 0)
         {
            throw std::runtime_error("Invalid STP file: The number of edges cannot be negative.");
         }
         return 1;
      }
      else if (_keyword == stp_graph_edge_keyword)
      {
         ss >> _head >> _tail >> _weight;
         _edge_counter++;
         if (_tail == _head)
         {
            throw std::runtime_error("Invalid STP file: Loops are not allowed.");
         }
         return 2;
      }
      return -1;
   }

   int terminals_section(const std::string &_line, int &_node, int &_num_terminals, unsigned int &_terminal_counter)
   {
      std::stringstream ss(_line);
      std::string _keyword = "";
      ss >> _keyword;
      if (_keyword == stp_terminals_terminals_keyword)
      {
         if (_num_terminals != -1)
         {
            throw std::runtime_error("Invalid STP file: Contains 'Terminals' keyword more than once.");
         }
         ss >> _num_terminals;
         if (_num_terminals < 0)
         {
            throw std::runtime_error("Invalid STP file: The number of terminals cannot be negative.");
         }
         return 0;
      }
      else if (_keyword == stp_terminals_terminal_keyword)
      {
         ss >> _node;
         _terminal_counter++;
         return 1;
      }
      return -1;
   }
}

const SteinerGraph::NodeId SteinerGraph::invalid_node = -1;
const double SteinerGraph::infinite_weight = std::numeric_limits<double>::max();

void SteinerGraph::add_nodes(NodeId num_new_nodes)
{
   _nodes.resize(num_nodes() + num_new_nodes);
}

void SteinerGraph::add_terminal(NodeId new_terminal)
{
   if (new_terminal >= num_nodes() or new_terminal < 0)
   {
      throw std::runtime_error("Terminalstatus of Node cannot be changed due to undefined node");
   }
   _nodes[new_terminal].set_terminal();
}

void SteinerGraph::Node::set_terminal()
{
   _terminal = true;
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

bool SteinerGraph::Node::is_terminal() const
{
   return _terminal;
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

SteinerGraph::SteinerGraph(char const *filename) // Konstruktor der Klasse   -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
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

   STPSection last_section = NoSection;
   STPSection current_section = NoSection;

   if (line != stp_control_line)
   {
      throw std::runtime_error("Invalid STP file: Does not start with STP control line.");
   }

   bool reached_section_graph = false;
   bool reached_eof = false;

   int num_nodes = -1, num_edges = -1, num_terminals = -1; // es fehlen noch die error-catches, falls die Zahlen zu groß werden, auch wenn das sehr unrealistisch ist...
   unsigned int edge_counter = 0, terminal_counter = 0;
   int head = -1, tail = -1;
   double weight = 1.0;
   std::string keyword = "";

   while (std::getline(file, line))
   {
      if (reached_eof)
      {
         throw std::runtime_error("Invalid STP file: Reached '" + stp_eof_line + "' before end of file.");
      }

      if (line == stp_eof_line)
      {
         if (current_section != NoSection)
         {
            throw std::runtime_error("Invalid STP file: Reached '" + stp_eof_line + "' before section was closed.");
         }
         reached_eof = true;
         continue;
      }

      if (current_section == NoSection) //                                                      check if section != NoSection has a new beginning of a Section, meaning, that an "END" was missing
      {
         if (!(check_for_sections(current_section, last_section, line)))
         {
            if (line != stp_empty_line)
            {
               throw std::runtime_error("Invalid STP file: Found non-empty line outside section.");
            }
         }
         continue;
      }
      else
      {
         if (check_for_sections(current_section, last_section, line))
         {
            throw std::runtime_error("Invalid STP file: Found beginning of new section inside of a section.");
         }
         if (current_section == CommentSection or current_section == MaximumDegreesSection or current_section == CoordinatesSection)
         {
            if (line == stp_end_line) // Checking if the Section ends here, since nothing is done with the contents
            {
               current_section = NoSection;
            }
            continue;
         }
         // bool graph_section(const std::string &_line, int &_num_nodes, int &_num_edges, int _head, int _tail) wobei true für nodes und false für einzelne hinzzufügende Kanten steht
         if (current_section == GraphSection)
         {
            if (line == stp_end_line)
            {
               if (num_edges == -1)
               {
                  throw std::runtime_error("Invalid STP file: The number of edges have not been specified.");
               }
               if (edge_counter != num_edges)
               {
                  throw std::runtime_error("Invalid STP file: The specified number of edges does not match the edgecount.");
               }
               if (num_nodes == -1)
               {
                  throw std::runtime_error("Invalid STP file: The number of nodes have not been specified.");
               }
               last_section = GraphSection;
               current_section = NoSection;
               continue;
            }
            else
            {
               int a = graph_section(line, num_nodes, num_edges, head, tail, weight, edge_counter);
               if (a == 0)
               {
                  add_nodes(num_nodes);
               }
               else if (a == 1 and tail != -1 and head != -1)
               {
                  add_edge(tail, head, weight);
                  head = -1;
                  tail = -1;
               }
               else if (a == -1)
               {
                  throw std::runtime_error("Invalid STP file: Invalid line in graphsection.");
               }
               continue;
            }
         }
         if (current_section == TerminalsSection)
         {
            int node = -1;
            if (line == stp_end_line)
            {
               if (num_terminals == -1)
               {
                  throw std::runtime_error("Invalid STP file: The number of terminals has not been specified.");
               }
               if (num_terminals != terminal_counter)
               {
                  throw std::runtime_error("Invalid STP file: The specified number of terminals does not match the terminalcount.");
               }
               last_section = TerminalsSection;
               current_section == NoSection;
            }
            else
            {
               int b = terminals_section(line, node, num_terminals, terminal_counter);
               if (b == 1)
               {
                  add_terminal(node);
               }
               else if (b == -1)
               {
                  throw std::runtime_error("Invalid STP file: Invalid line in terminalssection.");
               }
            }
         }
      }
   } // -   -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

   if (!reached_eof)
   {
      throw std::runtime_error("Invalid STP file: File does not end with '" + stp_eof_line + "'.");
   }

   /*std::stringstream ss(line); // convert line to a stringstream
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
   } */
}