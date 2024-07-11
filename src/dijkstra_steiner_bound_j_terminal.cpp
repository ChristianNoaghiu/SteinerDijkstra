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

    if (!_graph.get_node(r0).is_terminal())
    {
        throw std::runtime_error("r0 must be a terminal");
    }

    SteinerGraph::TerminalId r0_terminal_id = _graph.find_terminal_id(r0).value();
    if (terminal_subset[r0_terminal_id] == 0)
    {
        return 0;
    }

    // check if the bound has already been computed and return it
    const LabelKey label_key = std::make_pair(node, terminal_subset);
    if (_computed_j_terminal_bounds.count(label_key) > 0)
    {
        return _computed_j_terminal_bounds[label_key];
    }

    int result = 0;

    for (int subset_size = 1; subset_size <= std::min(j + 1, _graph.num_terminals()); subset_size++)
    {
        for (TerminalSubset terminal_subset_J : _terminal_subsets_of_size.at(subset_size))
        {
            // r0 must be in J
            if (terminal_subset_J[r0_terminal_id] == 0)
            {
                continue;
            }

            // check if J is a subset of I
            if (!is_terminal_subset_of(terminal_subset_J, terminal_subset))
            {
                continue;
            }

            // compute the SMT without v
            if (!_computed_smts_r0.has_value())
            {
                _computed_smts_r0 = r0;
            }
            else if (_computed_smts_r0.value() != r0)
            {
                throw std::runtime_error("r0 value does not match with previous computations");
            }

            int smt_without_node = 0;
            if (!_computed_smt_without_extra_node.count(terminal_subset_J))
            {
                smt_without_node = dijkstra_steiner_algorithm(_graph, r0, false, terminal_subset_J).edge_weight_sum();
                _computed_smt_without_extra_node[terminal_subset_J] = smt_without_node;
            }
            else
            {
                smt_without_node = _computed_smt_without_extra_node[terminal_subset_J];
            }

            if (smt_without_node > result)
            {
                result = smt_without_node;
            }

            // check if J united with {v} also has size <= j+1
            // this means that |J| = j+1 and v is not a terminal in J
            if (subset_size == j + 1)
            {
                if (!_graph.get_node(node).is_terminal() || !terminal_subset_J[_graph.find_terminal_id(node).value()])
                {
                    continue;
                }
            }

            // compute the SMT with v
            int smt_with_node = 0;
            LabelKey label_key_with_node = std::make_pair(node, terminal_subset_J);
            if (!_computed_smt_with_extra_node.count(label_key_with_node))
            {
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

                smt_with_node = dijkstra_steiner_algorithm(graph_with_node_as_terminal, r0, false, terminal_subset_J_with_node).edge_weight_sum();
                _computed_smt_with_extra_node[label_key_with_node] = smt_with_node;
            }
            else
            {
                smt_with_node = _computed_smt_with_extra_node[label_key_with_node];
            }
            if (smt_with_node > result)
            {
                result = smt_with_node;
            }
        }
    }

    _computed_j_terminal_bounds[label_key] = result;
    return result;
}
