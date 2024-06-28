#include "steinergraph.h"
#include <queue>

// leider viel duplicate code, aber die Datenstrukturen sind unterschiedlich
SteinerGraph::allDijkstraPathsStruct SteinerGraph::all_dijkstra_paths(const NodeId start_node) const
{
    // initialise result
    allDijkstraPathsStruct result;

    // initialise distances
    result.distances = std::vector<int>(num_nodes(), infinite_distance);
    result.distances.at(start_node) = 0;

    // initialise predecessors
    result.predecessors = std::vector<std::vector<std::pair<std::optional<NodeId>, int>>>(num_nodes(), {std::make_pair(std::nullopt, infinite_distance)});

    // initialise Dijkstra queue
    const auto compare_function = edge_tuple_compare();
    std::priority_queue<EdgeTuple, std::vector<EdgeTuple>, decltype(compare_function)>
        dijkstra_queue(compare_function);

    // stores whether a node has been removed from the Dijkstra queue
    std::vector<bool> visited(num_nodes(), false);

    // add the edges going out from the start node to the queue
    for (Neighbor neighbor : get_node(start_node).adjacent_nodes())
    {
        dijkstra_queue.push(std::make_tuple(start_node, neighbor.id(), neighbor.edge_weight()));
    }

    // iterate through the queue
    while (!dijkstra_queue.empty())
    {
        // take the node with the currently smallest distance
        const EdgeTuple current_node_distance = dijkstra_queue.top();
        const NodeId current_node = std::get<0>(current_node_distance);

        dijkstra_queue.pop();
        if (visited.at(current_node))
        {
            if (std::get<2>(current_node_distance) == result.distances.at(current_node))
            {
                result.predecessors.at(std::get<1>(current_node_distance)).push_back(std::make_pair(current_node, std::get<2>(current_node_distance) - result.distances.at(current_node)));
            }
            continue;
        }
        visited.at(current_node) = true;

        const int distance_to_current_node = result.distances.at(current_node);

        // iterate through his unvisited neighbors
        for (auto neighbor : get_node(current_node).adjacent_nodes())
        {
            const int distance_to_neighbor = result.distances.at(neighbor.id());
            if (visited.at(neighbor.id()))
            {
                if (distance_to_current_node + neighbor.edge_weight() == distance_to_neighbor)
                {
                    result.predecessors.at(neighbor.id()).push_back(std::make_pair(current_node, neighbor.edge_weight()));
                }
                continue;
            }

            // check if this results in a smaller distance from start_node
            if (distance_to_current_node + neighbor.edge_weight() < distance_to_neighbor)
            {
                // if yes, update the distance to the current neighbor
                const int updated_distance = distance_to_current_node + neighbor.edge_weight();
                result.distances.at(neighbor.id()) = updated_distance;
                result.predecessors.at(neighbor.id()).push_back(std::make_pair(current_node, neighbor.edge_weight()));
                // queue the neighbor
                // possibly it is already queued with a larger key,
                // but we have ensured above that no vertex is visited twice
                dijkstra_queue.push(std::make_tuple(neighbor.id(), current_node, updated_distance));
            }
        }
    }
    return result;
}
