// steinergraph_one_tree_bound.cpp (computing the 1-tree-bound from the Dijkstra-Steiner algorithm)
#include "dijkstra_steiner.h"
#include "steinergraph.h"
#include <optional>

/**
 * returns the 1-tree-bound from the Dijkstra-Steiner algorithm
 */
double DijkstraSteiner::get_or_compute_one_tree_bound(
    const SteinerGraph::NodeId node,
    const DijkstraSteiner::TerminalSubset &terminal_subset,
    const SteinerGraph::NodeId r0)
{
    // check validity of parameters
    _graph.check_valid_node(node);

    if (!_graph.get_node(r0).is_terminal())
    {
        throw std::runtime_error("r0 must be a terminal");
    }

    SteinerGraph::TerminalId r0_terminal_id = _graph.find_terminal_id(r0).value();
    // second case of one-tree bound definition
    if (terminal_subset[r0_terminal_id] == 0)
    {
        return 0.0;
    }

    // check if r0 is the same as in previous computations
    if (!_computed_one_tree_bound_root_terminal.has_value())
    {
        _computed_one_tree_bound_root_terminal = r0;
    }
    else if (_computed_one_tree_bound_root_terminal.value() != r0)
    {
        throw std::runtime_error("Root terminal of one-tree bound computation does not match.");
    }

    // check if the bound has already been computed and return it
    const LabelKey label_key = std::make_pair(node, terminal_subset);
    if (_computed_one_tree_bounds.count(label_key) > 0)
    {
        return _computed_one_tree_bounds[label_key];
    }

    const SteinerGraph mst_graph = _metric_closure_graph_result.subgraph_mst(is_in_terminal_subset(terminal_subset));
    const double mst_value = mst_graph.edge_weight_sum();

    // compute the minimum of d(v,i) + d(v,j) as in the definition of the one-tree bound
    int distance_sum = std::numeric_limits<int>::max();

    // iterate over all pairs of terminals
    for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
    {
        if (terminal_subset[i] == 0)
        {
            continue;
        }

        for (SteinerGraph::TerminalId j = i; j < _graph.num_terminals(); j++)
        {
            if (terminal_subset[j] == 0)
            {
                continue;
            }

            SteinerGraph::NodeId node_i = _graph.get_terminals().at(i);
            SteinerGraph::NodeId node_j = _graph.get_terminals().at(j);

            // if terminal_subset contains more than one node, the definition excludes the case i == j
            if (i == j && terminal_subset.count() > 1)
            {
                continue;
            }

            // compute sum of distances as in definition
            const int distance_node_i = get_or_compute_distance(node, node_i);
            const int distance_node_j = get_or_compute_distance(node, node_j);

            if (distance_node_j > std::numeric_limits<int>::max() - distance_node_i)
            {
                throw std::runtime_error("Distances too large, will provoke overflow.");
            }

            // for having the minimum in the end
            const int new_distance_sum = distance_node_i + distance_node_j;
            if (new_distance_sum < distance_sum)
            {
                distance_sum = new_distance_sum;
            }
        }
    }

    // store the result dynamically
    double result = double(static_cast<double>(distance_sum) / 2.0) + double(mst_value / 2.0);
    _computed_one_tree_bounds[label_key] = result;
    return result;
}
