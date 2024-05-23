// steinergraph_steiner_tree_mst_approximation.cpp (computing a steiner tree approximation using an mst in the metric closure)
#include "steinergraph.h"
#include <stdexcept>

/**
 * decodes the MST-metric-closure-path from a start_node into the real
 * path of the graph and adds all edges on the way to the result_graph
 * of the steiner_tree_mst_approximation
 */
void SteinerGraph::add_path_to_steiner_tree_mst_approximation(
    const SteinerGraph::NodeId &start_node,
    const std::vector<std::vector<std::optional<SteinerGraph::NodeId>>> &metric_closure_predecessor_matrix,
    const std::vector<std::vector<int>> &metric_closure_predecessor_weight_matrix,
    const SteinerGraph &mst_graph,
    std::vector<bool> &visited,
    SteinerGraph &result_graph)
    const
{
    const std::optional<NodeId> mst_predecessor_optional = mst_graph.get_node(start_node).get_predecessor();

    if (!mst_predecessor_optional.has_value())
    {
        return;
    }

    // start at start_node and go towards its mst predecessor
    // according to the shortest path
    const NodeId mst_predecessor = mst_predecessor_optional.value();
    NodeId node_on_path = start_node;

    while (node_on_path != mst_predecessor)
    {
        // if node_on_path has been visited, it is already
        // connected to mst_predecessor in the result graph,
        // thus we can stop
        if (visited.at(node_on_path))
        {
            return;
        }
        visited.at(node_on_path) = true;

        // add edge to next node on path with its weight to result graph
        const std::optional<NodeId> next_node_on_path_optional = metric_closure_predecessor_matrix.at(mst_predecessor).at(node_on_path);
        if (!next_node_on_path_optional.has_value())
        {
            throw std::runtime_error("Node has no predecessor in metric closure predecessor!");
        }

        const NodeId next_node_on_path = next_node_on_path_optional.value();
        const int edge_weight = metric_closure_predecessor_weight_matrix.at(node_on_path).at(next_node_on_path);
        result_graph.add_edge(node_on_path, next_node_on_path, edge_weight);

        node_on_path = next_node_on_path;
    }
}

/** computes a 2-approximation of a Steiner tree by
 * computing a MST on the terminal subgraph in the metric closure
 */
SteinerGraph SteinerGraph::steiner_tree_mst_approximation() const
{
    SteinerGraph result_graph = clear_edges();

    if (_terminals.size() == 0)
    {
        return result_graph;
    }

    // compute the metric closure and MST on it
    const MetricClosureStruct metric_closure_result = metric_closure();
    check_connected_metric_closure(metric_closure_result.distance_matrix);

    const SteinerGraph metric_closure_graph_result = metric_closure_graph(metric_closure_result.distance_matrix);
    const SteinerGraph mst_graph = metric_closure_graph_result.subgraph_mst(is_in_set(_terminals));

    std::vector<bool> visited(num_nodes(), false);

    // for every node, decode the MST-metric-closure-path into the shortest path of the graph
    // and add all the edges on the way to the result
    // (add_path_to_steiner_tree_mst_approximation does not do anything
    // if path_start_node is not a terminal)
    for (NodeId path_start_node = 0; path_start_node < num_nodes(); path_start_node++)
    {
        add_path_to_steiner_tree_mst_approximation(
            path_start_node,
            metric_closure_result.predecessor_matrix,
            metric_closure_result.predecessor_weight_matrix,
            mst_graph,
            visited,
            result_graph);
    }

    // return a MST in the result graph
    // terminal_node must have a value since _terminals.size() == 0 has been excluded above
    const NodeId terminal_node = find_terminal_node().value();
    return result_graph.component_mst(terminal_node);
}