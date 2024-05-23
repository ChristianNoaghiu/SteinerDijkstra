// steinergraph_mst.cpp (methods for computing MSTs)
#include <stdexcept>
#include <queue>
#include "steinergraph.h"

/**
 * computes a MST the subgraph induced by is_in_subgraph
 * using Prim's algorithm (in the connected component of start_node)
 * and saving the predecessor on the path towards start_node
 */
SteinerGraph SteinerGraph::subgraph_mst(
    const std::function<bool(const NodeId node)> is_in_subgraph,
    const NodeId start_node)
    const
{
    if (num_nodes() == 0)
    {
        throw std::runtime_error("Graph has no vertices.");
    }

    check_valid_node(start_node);

    if (!is_in_subgraph(start_node))
    {
        throw std::runtime_error("start_node is not in subgraph");
    }

    SteinerGraph result_graph = clear_edges();

    // initialise the distances and predecessors
    std::vector<int> distances(num_nodes(), infinite_distance);
    distances.at(start_node) = 0;

    std::vector<std::optional<NodeId>> predecessors(num_nodes(), std::nullopt);

    // stores whether a node has been removed from the Prim queue
    std::vector<bool> visited(num_nodes(), false);

    const auto compare_function = node_distance_pair_compare();
    std::priority_queue<NodeDistancePair, std::vector<NodeDistancePair>, decltype(compare_function)>
        prim_queue(compare_function);

    prim_queue.push(std::make_pair(start_node, distances.at(start_node)));

    // pick the node with the smallest edge leaving the current result subgraph
    while (!prim_queue.empty())
    {
        const NodeDistancePair current_node_distance = prim_queue.top();
        const NodeId current_node = current_node_distance.first;
        prim_queue.pop();

        // skip nodes that are not in the subgraph
        if (!is_in_subgraph(current_node))
        {
            continue;
        }

        if (visited.at(current_node))
        {
            // ensure that no vertex is visited twice
            // (vertices can occur multiple times in the queue with
            // different keys)
            continue;
        }
        visited.at(current_node) = true;

        if (current_node != start_node)
        {
            std::optional<NodeId> predecessor_optional = predecessors.at(current_node);
            if (!predecessor_optional.has_value())
            {
                throw std::runtime_error("current_node has no predecessor in component_mst");
            }

            NodeId predecessor = predecessor_optional.value();
            result_graph.add_edge(current_node, predecessor, distances.at(current_node));
            result_graph.set_predecessor(current_node, predecessor_optional);
        }

        // iterate through all unvisited neighbors that are terminals
        for (const Neighbor &neighbor : get_node(current_node).adjacent_nodes())
        {
            const NodeId neighbor_id = neighbor.id();

            if (visited.at(neighbor_id))
            {
                continue;
            }

            // check if the corresponding edge would lower the distance to the current node
            const int distance_to_neighbor = distances.at(neighbor_id);
            const int edge_weight_to_neighbor = neighbor.edge_weight();

            if (edge_weight_to_neighbor < distance_to_neighbor)
            {
                // if yes, update the neighbor's distance
                distances.at(neighbor_id) = edge_weight_to_neighbor;
                predecessors.at(neighbor_id) = current_node;

                prim_queue.push(std::make_pair(neighbor_id, edge_weight_to_neighbor));
            }
        }
    }

    return result_graph;
}

/**
 * computes a MST in the subgraph induced by is_in_subgraph
 * using Prim's algorithm (in the connected component of
 * a random node)
 */
SteinerGraph SteinerGraph::subgraph_mst(
    const std::function<bool(const NodeId node)> is_in_subgraph)
    const
{
    std::optional<NodeId> start_node = {};

    /**
     * iterate through all the nodes until a node in the subgraph
     * is found
     */
    for (NodeId node = 0; node < num_nodes(); node++)
    {
        if (is_in_subgraph(node))
        {
            start_node = node;
            break;
        }
    }

    if (!start_node.has_value())
    {
        throw std::runtime_error("Subgraph has no vertices.");
    }

    return subgraph_mst(is_in_subgraph, start_node.value());
}

/**
 * computes a MST in the connected component of start_node
 * using Prim's algorithm
 */
SteinerGraph SteinerGraph::component_mst(
    const NodeId start_node)
    const
{
    return subgraph_mst(is_in_graph(), start_node);
}