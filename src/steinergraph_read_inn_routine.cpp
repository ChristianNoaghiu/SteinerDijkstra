#include <fstream>
#include <sstream>
#include <vector>
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
    const std::string stp_empty_line = "";

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

    void section_comparison(const STPSection &_section, const STPSection &_last_section)
    {
        if (_section <= _last_section)
        {
            throw std::runtime_error("Invalid STP file: Sections are not ordered correctly.");
        }
    }

    bool check_for_sections(STPSection &_section, STPSection &_last_section, const std::string &_line) // checking for beginnings of sections and checking if the order is right
    {
        if (_line == stp_section_comment_line)
        {
            _section = CommentSection;
            section_comparison(_section, _last_section);
            _last_section = CommentSection; // This is only featured in the sections where nothing is done, since we don't differentiate them in the constructor
            return true;
        }
        else if (_line == stp_section_graph_line)
        {
            _section = GraphSection;
            section_comparison(_section, _last_section);
            return true;
        }
        else if (_line == stp_section_terminals_line)
        {
            _section = TerminalsSection;
            section_comparison(_section, _last_section);
            return true;
        }
        else if (_line == stp_section_maximumdegrees_line)
        {
            _section = MaximumDegreesSection;
            section_comparison(_section, _last_section);
            _last_section = MaximumDegreesSection;
            return true;
        }
        else if (_line == stp_section_coordinates_line)
        {
            _section = CoordinatesSection;
            section_comparison(_section, _last_section);
            _last_section = CoordinatesSection;
            return true;
        }
        return false;
    }

    /**

     */
    SteinerGraph graph_section(std::ifstream &file)
    {
        std::string line;
        std::getline(file, line);
        std::stringstream ss(line);
        std::string _keyword = "";
        ss >> _keyword;

        int num_nodes = -1, num_edges = -1, head = -1, tail = -1, weight = 1, edge_counter = 0;
        SteinerGraph graph = SteinerGraph(0);

        while (_keyword != stp_end_line)
        {
            if (_keyword == stp_graph_nodes_keyword)
            {
                if (num_nodes != -1)
                {
                    throw std::runtime_error("Invalid STP file: Contains 'Nodes' keyword more than once.");
                }
                ss >> num_nodes;
                if (num_nodes < 0)
                {
                    throw std::runtime_error("Invalid STP file: The number of nodes cannot be negative.");
                }
                graph.add_nodes(num_nodes);
            }
            else if (_keyword == stp_graph_edges_keyword)
            {
                if (num_edges != -1)
                {
                    throw std::runtime_error("Invalid STP file: Contains 'Nodes' keyword more than once.");
                }
                ss >> num_edges;
                if (num_edges < 0)
                {
                    throw std::runtime_error("Invalid STP file: The number of edges cannot be negative.");
                }
            }
            else if (_keyword == stp_graph_edge_keyword)
            {
                ss >> head >> tail >> weight;
                edge_counter++;
                if (tail == head)
                {
                    throw std::runtime_error("Invalid STP file: Loops are not allowed.");
                }
                if (tail < 0 or head < 0)
                {
                    throw std::runtime_error("Invalid STP file: Edge-endpoints need to be defined/be greater or equal to zero.");
                }
                graph.add_edge(tail, head, weight);
            }
            else
            {
                throw std::runtime_error("Invalid STP file: Invalid line in graphsection.");
            }
            std::getline(file, line);
            std::stringstream ss(line);
            ss >> _keyword;
        }
        if (num_edges != edge_counter)
        {
            throw std::runtime_error("Invalid STP file: The specified number of edges does not match the edgecount.");
        }
        if (num_nodes <= 0)
        {
            throw std::runtime_error("Invalid STP file: The number of nodes have not been specified/are negative/are zero.");
        }
        if (num_edges == -1)
        {
            throw std::runtime_error("Invalid STP file: The number of edges have not been specified.");
        }
        return graph;
    }
    /**
     * Checking for lines belonging to the terminals-section.
     * The output-number tells which kind of information was in the line given.
     */
    std::vector<int> terminals_section(std::ifstream &file)
    {
        std::string line;
        std::stringstream ss(line);
        std::string _keyword = "";
        ss >> _keyword;

        std::vector<int> terminals;
        int num_terminals = -1, _node = -1, terminal_counter = 0;

        while (_keyword != stp_end_line)
        {
            if (_keyword == stp_terminals_terminals_keyword)
            {
                if (num_terminals != -1)
                {
                    throw std::runtime_error("Invalid STP file: Contains 'Terminals' keyword more than once.");
                }
                ss >> num_terminals;
                if (num_terminals < 0)
                {
                    throw std::runtime_error("Invalid STP file: The number of terminals cannot be negative.");
                }
            }
            else if (_keyword == stp_terminals_terminal_keyword)
            {
                ss >> _node;
                terminals.push_back(_node);
                terminal_counter++;
            }
            else
            {
                throw std::runtime_error("Invalid STP file: Invalid line in terminalssection.");
            }
            std::getline(file, line);
            std::stringstream ss(line);
            ss >> _keyword;
        }
        if (num_terminals != terminal_counter)
        {
            throw std::runtime_error("Invalid STP file: The specified number of terminals does not match the terminalcount.");
        }
        return terminals;
    }

    SteinerGraph::SteinerGraph(char const *filename) // Konstruktor der Klasse   -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    {
        std::ifstream file(filename); // open file
        if (not file)
        {
            throw std::runtime_error("Cannot open file.");
        }

        // SteinerGraph::NodeId num = 0;
        std::string line;
        std::getline(file, line); // get first line of file
        // std::cout << line << std::endl;

        STPSection last_section = NoSection; // Variables to save where we are in the file and if a new section is in the right order
        STPSection current_section = NoSection;

        if (line != stp_control_line)
        {
            throw std::runtime_error("Invalid STP file: Does not start with STP control line.");
        }

        bool reached_eof = false;

        int num_terminals = -1;
        std::string keyword = "";

        while (std::getline(file, line)) // while-loop that reads the file line by line
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

            if (current_section == NoSection) // Checking if a new Section begins here
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
                if (check_for_sections(current_section, last_section, line)) // Checking if a Section wasn't closed, since a new Section was opened inside another one
                {
                    throw std::runtime_error("Invalid STP file: Found beginning of new section inside of a section.");
                }
                if (current_section == CommentSection or current_section == MaximumDegreesSection or current_section == CoordinatesSection)
                {
                    if (line == stp_end_line) // Checking if the Section ends here and skip if not, since nothing is done with the contents
                    {
                        current_section = NoSection;
                    }
                    continue;
                }
                if (current_section == GraphSection) // read-in routine for the graph section
                {
                    SteinerGraph temp = graph_section(file);
                    *this = temp;
                    last_section = GraphSection;
                    current_section = NoSection;
                }
                if (current_section == TerminalsSection)
                {
                    std::vector<int> terminals = terminals_section(file);
                    for (int terminal_it : terminals)
                    {
                        make_terminal(terminal_it);
                    }
                    last_section = TerminalsSection;
                    current_section = NoSection;
                }
            }
        } // -   -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        if (!reached_eof)
        {
            throw std::runtime_error("Invalid STP file: File does not end with '" + stp_eof_line + "'.");
        }
    }