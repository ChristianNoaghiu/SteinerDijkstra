#include <stdexcept>
#include <queue>
#include <functional>
#include "steinergraph.h"

namespace
{
    using NodeDistancePair = std::pair<SteinerGraph::NodeId, int>;

    // returns a lambda which compares the distances of two NodeDistancePairs
    // according to the second component

    std::function<bool(
        const NodeDistancePair,
        const NodeDistancePair)>
    node_distance_pair_compare()
    {
        return [](
                   const NodeDistancePair &node_distance1,
                   const NodeDistancePair &node_distance2)
        {
            const int distance1 = node_distance1.second;
            const int distance2 = node_distance2.second;

            const bool is_node1_infinite = (distance1 == SteinerGraph::infinite_distance);
            const bool is_node2_infinite = (distance2 == SteinerGraph::infinite_distance);

            const bool are_both_nodes_finite = (!is_node1_infinite && !is_node2_infinite);
            const bool is_node1_bigger_node2 = (distance1 > distance2);

            return (is_node1_infinite && !is_node2_infinite) ||
                   (are_both_nodes_finite && is_node1_bigger_node2);
        };
    }
}

// computes distances to every node and stores these as well as
// the predecessors and their edge weights on the respective path in distances/predecessors
void SteinerGraph::dijkstra(
    const NodeId start_node,
    std::vector<int> &distances,
    std::vector<NodeId> &predecessors,
    std::vector<int> &predecessor_weights)
    const
{
    // initialise distances
    distances = std::vector<int>(num_nodes(), infinite_distance);
    distances.at(start_node) = 0;

    // initialise predecessors
    predecessors = std::vector<int>(num_nodes(), invalid_node);
    predecessor_weights = std::vector<int>(num_nodes(), infinite_weight);

    auto compare_function = node_distance_pair_compare();
    std::priority_queue<NodeDistancePair, std::vector<NodeDistancePair>, decltype(compare_function)>
        dijkstra_queue(compare_function);

    // stores whether a node has been removed from the Dijkstra queue
    std::vector<bool> visited(num_nodes(), false);

    dijkstra_queue.push(std::make_pair(start_node, distances.at(start_node)));

    while (!dijkstra_queue.empty())
    {
        // take the node with the currently smallest distance
        const NodeDistancePair current_node_distance = dijkstra_queue.top();
        const NodeId current_node = current_node_distance.first;

        dijkstra_queue.pop();
        if (visited.at(current_node))
        {
            // ensure that no vertex is visited twice
            // (vertices can occur multiple times in the queue with
            // different keys)
            continue;
        }
        visited.at(current_node) = true;

        const int distance_to_current_node = distances.at(current_node);

        // iterate through his unvisited neighbors
        for (auto neighbor : get_node(current_node).adjacent_nodes())
        {
            if (visited.at(neighbor.id()))
            {
                continue;
            }

            const int distance_to_neighbor = distances.at(neighbor.id());

            // check if this results in a smaller distance from start_node
            if (distance_to_neighbor == infinite_distance ||
                distance_to_current_node + neighbor.edge_weight() < distance_to_neighbor)
            {
                // if yes, update the distance to the current neighbor
                int updated_distance = distance_to_current_node + neighbor.edge_weight();
                distances.at(neighbor.id()) = updated_distance;
                predecessors.at(neighbor.id()) = current_node;
                predecessor_weights.at(neighbor.id()) = neighbor.edge_weight();

                // queue the neighbor
                // possibly it is already queued with a larger key,
                // but we have ensured above that no vertex is visited twice
                dijkstra_queue.push(std::make_pair(neighbor.id(), updated_distance));
            }
        }
    }
}

// computes a MST in the connected component of start_node
// using Prim's algorithm
SteinerGraph SteinerGraph::component_mst(
    const NodeId start_node)
    const
{
    if (num_nodes() == 0)
    {
        throw std::runtime_error("Graph has no vertices.");
    }

    if (start_node < 0 || start_node >= num_nodes())
    {
        throw std::runtime_error("Invalid start_node");
    }

    SteinerGraph result_graph = clear_edges();

    // initialise the distances and predecessors
    std::vector<int> distances(num_nodes(), infinite_distance);
    distances.at(start_node) = 0;

    std::vector<NodeId> predecessors(num_nodes(), invalid_node);

    // stores whether a node has been removed from the Prim queue
    std::vector<bool> visited(num_nodes(), false);

    auto compare_function = node_distance_pair_compare();
    std::priority_queue<NodeDistancePair, std::vector<NodeDistancePair>, decltype(compare_function)>
        prim_queue(compare_function);

    prim_queue.push(std::make_pair(start_node, distances.at(start_node)));

    // pick the node with the smallest edge leaving the current result subgraph
    while (!prim_queue.empty())
    {
        const NodeDistancePair current_node_distance = prim_queue.top();
        const NodeId current_node = current_node_distance.first;
        prim_queue.pop();

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
            result_graph.add_edge(current_node, predecessors.at(current_node), distances.at(current_node));
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

            if (distance_to_neighbor == infinite_distance ||
                edge_weight_to_neighbor < distance_to_neighbor)
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

// computes the metric closure of a graph
// stores the resulting distances in distance_matrix
// such that distance_matrix.at(i).at(j) is the distance of nodes i and j
// stores the predecessors on the corresponding paths in predecessor_matrix
// such that predecessor_matrix.at(i).at(j) is the predecessor
// of j on the path from i to j
// stores the corresponding edge weights in the path as well
void SteinerGraph::metric_closure(
    std::vector<std::vector<int>> &distance_matrix,
    std::vector<std::vector<NodeId>> &predecessor_matrix,
    std::vector<std::vector<int>> &predecessor_distance_matrix)
    const
{
    // initialise the result matrices
    distance_matrix = std::vector<std::vector<int>>();
    predecessor_matrix = std::vector<std::vector<NodeId>>();
    predecessor_distance_matrix = std::vector<std::vector<int>>();

    // perform Dijkstra from every node
    for (int i = 0; i < num_nodes(); i++)
    {
        std::vector<int> distances_from_i;
        std::vector<NodeId> predecessors_to_i;
        std::vector<int> predecessor_weights_to_i;

        dijkstra(i, distances_from_i, predecessors_to_i, predecessor_weights_to_i);

        // add the results to the result matrices
        distance_matrix.push_back(distances_from_i);
        predecessor_matrix.push_back(predecessors_to_i);
        predecessor_distance_matrix.push_back(predecessor_weights_to_i);
    }
}

// returns a terminal node
SteinerGraph::NodeId SteinerGraph::find_terminal_node() const
{
    if (_terminals.size() == 0)
    {
        return invalid_node;
    }

    return *_terminals.begin();
}

void SteinerGraph::check_connected_metric_closure(
    const std::vector<std::vector<int>> &metric_closure_distance_matrix)
    const
{
    // check whether graph is connected (i.e. no distance is infinite)
    for (NodeId node1 = 0; node1 < num_nodes(); node1++)
    {
        for (NodeId node2 = 0; node2 < num_nodes(); node2++)
        {
            if (metric_closure_distance_matrix.at(node1).at(node2) == infinite_distance)
            {
                throw std::runtime_error("Graph is not connected.");
            }
        }
    }
}

// computes a MST on the terminal subgraph in the metric closure using Prim's algorithm
// and stores the predecessors of a corresponding rooted arborescence in predecessors
void SteinerGraph::terminal_rooted_mst(
    const std::vector<std::vector<int>> &metric_closure_distance_matrix,
    std::vector<NodeId> &predecessors)
    const
{
    check_connected_metric_closure(metric_closure_distance_matrix);

    // pick a start node for Prim
    const NodeId start_node = find_terminal_node();

    if (start_node == invalid_node)
    {
        throw std::runtime_error("No terminal node exists.");
    }

    // initialise the distances and predecessors
    std::vector<int> distances(num_nodes(), infinite_distance);
    distances.at(start_node) = 0;

    predecessors = std::vector<NodeId>(num_nodes(), invalid_node);

    // stores whether a node has been removed from the Prim queue
    std::vector<bool> visited(num_nodes(), false);

    auto compare_function = node_distance_pair_compare();
    std::priority_queue<NodeDistancePair, std::vector<NodeDistancePair>, decltype(compare_function)>
        prim_queue(compare_function);

    prim_queue.push(std::make_pair(start_node, distances.at(start_node)));

    // pick the node with the smallest edge leaving the current result subgraph
    while (!prim_queue.empty())
    {
        const NodeDistancePair current_node_distance = prim_queue.top();
        const NodeId current_node = current_node_distance.first;
        prim_queue.pop();

        if (visited.at(current_node))
        {
            // ensure that no vertex is visited twice
            // (vertices can occur multiple times in the queue with
            // different keys)
            continue;
        }
        visited.at(current_node) = true;

        // iterate through all unvisited neighbors that are terminals
        for (NodeId neighbor_id = 0; neighbor_id < num_nodes(); neighbor_id++)
        {
            if (!get_node(neighbor_id).is_terminal())
            {
                continue;
            }

            if (visited.at(neighbor_id))
            {
                continue;
            }

            // check if the corresponding edge would lower the distance to the current node
            const int distance_to_neighbor = distances.at(neighbor_id);
            const int edge_weight_to_neighbor = metric_closure_distance_matrix.at(current_node).at(neighbor_id);

            if (distance_to_neighbor == infinite_distance ||
                edge_weight_to_neighbor < distance_to_neighbor)
            {
                // if yes, update the neighbor's distance
                distances.at(neighbor_id) = edge_weight_to_neighbor;
                predecessors.at(neighbor_id) = current_node;

                prim_queue.push(std::make_pair(neighbor_id, edge_weight_to_neighbor));
            }
        }
    }
}

// decodes the MST-metric-closure-path from a start_node into the real
// path of the graph and adds all edges on the way to the result_graph
// of the steiner_tree_mst_approximation
void SteinerGraph::add_path_to_steiner_tree_mst_approximation(
    const SteinerGraph::NodeId &start_node,
    const std::vector<std::vector<SteinerGraph::NodeId>> &metric_closure_predecessor_matrix,
    const std::vector<std::vector<int>> &metric_closure_predecessor_weight_matrix,
    const std::vector<SteinerGraph::NodeId> &mst_predecessors,
    std::vector<bool> &visited,
    SteinerGraph &result_graph)
    const
{
    if (mst_predecessors.at(start_node) == invalid_node)
    {
        return;
    }

    // start at start_node and go towards its mst predecessor
    // according to the shortest path
    NodeId mst_predecessor = mst_predecessors.at(start_node);
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
        NodeId next_node_on_path = metric_closure_predecessor_matrix.at(mst_predecessor).at(node_on_path);
        int edge_weight = metric_closure_predecessor_weight_matrix.at(node_on_path).at(next_node_on_path);
        result_graph.add_edge(node_on_path, next_node_on_path, edge_weight);

        node_on_path = next_node_on_path;
    }
}

// computes a 2-approximation of a Steiner tree by
// computing a MST on the terminal subgraph in the metric closure
SteinerGraph SteinerGraph::steiner_tree_mst_approximation() const
{
    SteinerGraph result_graph = clear_edges();

    if (_terminals.size() == 0)
    {
        return result_graph;
    }

    // compute the metric closure and MST on it
    std::vector<std::vector<int>> metric_closure_distance_matrix(num_nodes(), std::vector<int>(num_nodes()));

    std::vector<std::vector<NodeId>> metric_closure_predecessor_matrix(num_nodes(), std::vector<NodeId>(num_nodes()));

    std::vector<std::vector<int>> metric_closure_predecessor_weight_matrix(num_nodes(), std::vector<int>(num_nodes(), infinite_weight));

    metric_closure(metric_closure_distance_matrix, metric_closure_predecessor_matrix, metric_closure_predecessor_weight_matrix);

    std::vector<NodeId> mst_predecessors(num_nodes(), invalid_node);
    terminal_rooted_mst(metric_closure_distance_matrix, mst_predecessors);

    std::vector<bool> visited(num_nodes(), false);

    // for every node, decode the MST-metric-closure-path into the shortest path of the graph
    // and add all the edges on the way to the result
    // (add_path_to_steiner_tree_mst_approximation does not do anything
    // if path_start_node is not a terminal)
    for (NodeId path_start_node = 0; path_start_node < num_nodes(); path_start_node++)
    {
        add_path_to_steiner_tree_mst_approximation(
            path_start_node,
            metric_closure_predecessor_matrix,
            metric_closure_predecessor_weight_matrix,
            mst_predecessors,
            visited,
            result_graph);
    }

    // return a MST in the result graph
    return result_graph.component_mst(find_terminal_node());
}