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

            // check if J is a subset of I
            if (!is_terminal_subset_of(terminal_subset_J, terminal_subset))
            {
                continue;
            }

            for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
            {

                double smt_without_node = smt(terminal_subset_J);
                if (smt_without_node > result)
                {
                    result = smt_without_node;
                }

                TerminalSubset terminal_subset_J_with_node = terminal_subset_J;
            }
        }
    }

    _computed_j_terminal_bounds[label_key] = result;
    return result;
}