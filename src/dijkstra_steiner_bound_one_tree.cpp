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
    _graph.check_valid_terminal(r0);

    if (r0 >= static_cast<int>(terminal_subset.size()))
    {
        throw std::runtime_error("r0 exceeds the size of terminal_subset");
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

    compute_distances_and_check_connected();

    /** @todo move this to separate algorithm class
     * there, we could also store the metric_closure_graph
     * (which is not possible here since SteinerGraph cannot have
     * a SteinerGraph as an attribute)
     */
    const SteinerGraph metric_closure_graph_result = _graph.metric_closure_graph(_distance_matrix);
    const SteinerGraph mst_graph = metric_closure_graph_result.subgraph_mst(is_in_terminal_subset(terminal_subset));
    double mst_value = mst_graph.edge_weight_sum();

    // compute the minimum of d(v,i) + d(v,j) as in the definition of the one-tree bound
    int distance_sum = std::numeric_limits<int>::max();

    // iterate over all pairs of terminals
    /** @todo maybe do this by index shifting */
    for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
    {
        if (terminal_subset[i] == 0)
        {
            continue;
        }

        for (SteinerGraph::TerminalId j = 0; j < _graph.num_terminals(); j++)
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
            /** @todo do this dynamically */
            int distance_node_i = get_or_compute_distance(node, node_i);
            int distance_node_j = get_or_compute_distance(node, node_j);

            if (distance_node_j > std::numeric_limits<int>::max() - distance_node_i)
            {
                throw std::runtime_error("Distances too large, will provoke overflow.");
            }

            // for having the minimum in the end
            int new_distance_sum = distance_node_i + distance_node_j;
            if (new_distance_sum < distance_sum)
            {
                distance_sum = new_distance_sum;
            }
        }
    }

    // store the result dynamically
    double result = (static_cast<double>(distance_sum) / 2) + (static_cast<double>(mst_value) / 2);
    _computed_one_tree_bounds[label_key] = result;
    return result;
}

/** @todo remove this */
void DijkstraSteiner::test_one_tree_bound()
{
    /*double x1 = get_or_compute_one_tree_bound(0, 0b111, 0);
    double x2 = get_or_compute_one_tree_bound(0, 0b111, 0);*/
    std::cout << get_or_compute_distance(0, 1) << "\n";
    std::cout << get_or_compute_distance(2, 3) << "\n";
    std::cout << get_or_compute_one_tree_bound(0, 0b011, 0) << "\n";
    std::cout << get_or_compute_one_tree_bound(1, 0b011, 0) << "\n";
    std::cout << get_or_compute_one_tree_bound(2, 0b011, 0) << "\n";
    std::cout << get_or_compute_one_tree_bound(3, 0b011, 0) << "\n";
    std::cout << get_or_compute_one_tree_bound(4, 0b011, 0) << "\n";
    std::cout << get_or_compute_one_tree_bound(0, 0b001, 0) << "\n";
}