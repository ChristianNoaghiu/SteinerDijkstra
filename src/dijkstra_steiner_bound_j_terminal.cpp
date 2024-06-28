// dijkstra_steiner_bound_j_terminal.cpp (computing the j-terminal bound from the Dijkstra-Steiner algorithm)
#include "dijkstra_steiner.h"
#include "steinergraph.h"
#include <optional>

int smt(__attribute__((unused)) DijkstraSteiner::TerminalSubset terminal_subset)
{
    return 0;
}

/**
 * returns the j-terminal bound
 */
double DijkstraSteiner::get_or_compute_j_terminal_bound(
    const int j,
    const SteinerGraph::NodeId r0,
    const SteinerGraph::NodeId node,
    const DijkstraSteiner::TerminalSubset &terminal_subset)
{
    _graph.check_valid_node(node);

    if (j < 1)
    {
        throw std::runtime_error("Invalid j value for j-terminal bound.");
    }

    if (!_computed_j_terminal_bound_j_value.has_value())
    {
        _computed_j_terminal_bound_j_value = j;
    }
    else if (_computed_j_terminal_bound_j_value.value() != j)
    {
        throw std::runtime_error("j value does not match with previous computations");
    }

    if (!_graph.get_node(r0).is_terminal() || !terminal_subset[r0])
    {
        return 0;
    }

    // check if the bound has already been computed and return it
    const LabelKey label_key = std::make_pair(node, terminal_subset);
    if (_computed_j_terminal_bounds.count(label_key) > 0)
    {
        return _computed_j_terminal_bounds[label_key];
    }

    /** @todo outsource this to avoid duplicate code */
    // terminal_subsets_of_size.at(n) stores all ullongs of terminal subsets of size n
    std::vector<std::vector<unsigned long long>> terminal_subsets_of_size(_graph.num_terminals() + 1);

    /** @todo what if num_terminals exceeds 64? Or is exactly 64? */
    for (unsigned long long terminal_subset_ullong = 0; terminal_subset_ullong < (1ULL << _graph.num_terminals()); terminal_subset_ullong++)
    {
        terminal_subsets_of_size.at(TerminalSubset(terminal_subset_ullong).count()).push_back(terminal_subset_ullong);
    }

    int result = 0;

    for (int subset_size = 1; subset_size <= j + 1; subset_size++)
    {
        for (unsigned long long terminal_subset_J_ullong : terminal_subsets_of_size.at(subset_size))
        {
            // create the corresponding TerminalSubset
            TerminalSubset terminal_subset_J = terminal_subset_J_ullong;

            // r0 must be in J
            if (terminal_subset_J[r0] == 0)
            {
                continue;
            }

            // check if J is a subset of I
            if (!is_terminal_subset_of(terminal_subset_J, terminal_subset))
            {
                continue;
            }

            SteinerGraph::NodeId r0_node_id = _graph.get_terminals().at(r0);

            // compute the SMT without v
            int smt_without_node = dijkstra_steiner_algorithm(_graph, r0_node_id, false, terminal_subset_J).edge_weight_sum();
            if (smt_without_node > result)
            {
                result = smt_without_node;
            }

            // if v is already in J, then we don't need to compute the SMT with v
            if (_graph.get_node(node).is_terminal() && terminal_subset_J[_graph.find_terminal_id(node).value()])
            {
                continue;
            }

            // create a graph where v is a terminal node
            SteinerGraph graph_with_node_as_terminal = _graph;
            if (!graph_with_node_as_terminal.get_node(node).is_terminal())
            {
                graph_with_node_as_terminal.make_terminal(node);
            }

            // create a new terminalsubset for J united with {v}
            const SteinerGraph::TerminalId node_terminal_id = graph_with_node_as_terminal.find_terminal_id(node).value();
            TerminalSubset terminal_subset_J_with_node = terminal_subset_J;
            terminal_subset_J_with_node[node_terminal_id] = 1;

            // compute the SMT with v
            int smt_with_node = dijkstra_steiner_algorithm(graph_with_node_as_terminal, r0_node_id, false, terminal_subset_J_with_node).edge_weight_sum();
            if (smt_with_node > result)
            {
                result = smt_with_node;
            }
        }
    }

    _computed_j_terminal_bounds[label_key] = result;
    return result;
}