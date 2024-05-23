// steinergraph_one_tree_bound.cpp (computing the 1-tree-bound from the Dijkstra-Steiner algorithm)
#include "steinergraph.h"

/**
 * returns the 1-tree-bound from the Dijkstra-Steiner algorithm
 */
double SteinerGraph::one_tree_bound(
    const NodeId node,
    const std::set<NodeId> &node_set,
    const NodeId r0)
    const
{
    // check validity of parameters
    check_valid_node(node);

    if (!get_node(r0).is_terminal())
    {
        throw std::runtime_error("r0 is not a terminal node");
    }

    for (auto node : node_set)
    {
        if (!get_node(node).is_terminal())
        {
            throw std::runtime_error("node_set is not a subset of terminal set");
        }
    }

    // second case of one-tree bound definition
    if (node_set.count(r0) == 0)
    {
        return 0.0;
    }

    // compute the metric closure and subgraph MST on it
    const MetricClosureStruct metric_closure_result = metric_closure();
    const std::vector<std::vector<int>> &distance_matrix = metric_closure_result.distance_matrix;
    check_connected_metric_closure(distance_matrix);

    /** @todo do this dynamically */
    const SteinerGraph metric_closure_graph_result = metric_closure_graph(distance_matrix);
    const SteinerGraph mst_graph = metric_closure_graph_result.subgraph_mst(is_in_set(node_set));
    double mst_value = mst_graph.edge_weight_sum();

    // compute the minimum of d(v,i) + d(v,j) as in the definition of the one-tree bound
    int distance_sum = std::numeric_limits<int>::max();

    for (auto i : node_set)
    {
        for (auto j : node_set)
        {
            if (i == j && node_set.size() > 1)
            {
                continue;
            }

            /** @todo do this dynamically */
            int distance_node_i = distance_matrix.at(node).at(i);
            int distance_node_j = distance_matrix.at(node).at(j);

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

    return (static_cast<double>(distance_sum) / 2) + (static_cast<double>(mst_value) / 2);
}