// steinergraph_metric_closure.cpp (methods for computing and handling the metric closure)
#include "steinergraph.h"
#include <queue>

/**
 * computes distances to every node and stores these as well as
 * the predecessors and their edge weights on the respective path in distances/predecessors
 */
SteinerGraph::DijkstraStruct SteinerGraph::dijkstra(
    const NodeId start_node)
    const
{
    // initialise distances
    DijkstraStruct result;

    result.distances = std::vector<int>(num_nodes(), infinite_distance);
    result.distances.at(start_node) = 0;

    // initialise predecessors
    result.predecessors = std::vector<std::optional<NodeId>>(num_nodes(), std::nullopt);
    result.predecessor_weights = std::vector<int>(num_nodes(), infinite_weight);

    const auto compare_function = node_distance_pair_compare();
    std::priority_queue<NodeDistancePair, std::vector<NodeDistancePair>, decltype(compare_function)>
        dijkstra_queue(compare_function);

    // stores whether a node has been removed from the Dijkstra queue
    std::vector<bool> visited(num_nodes(), false);

    dijkstra_queue.push(std::make_pair(start_node, result.distances.at(start_node)));

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

        const int distance_to_current_node = result.distances.at(current_node);

        // iterate through his unvisited neighbors
        for (auto neighbor : get_node(current_node).adjacent_nodes())
        {
            if (visited.at(neighbor.id()))
            {
                continue;
            }

            const int distance_to_neighbor = result.distances.at(neighbor.id());

            // check if this results in a smaller distance from start_node
            if (distance_to_current_node + neighbor.edge_weight() < distance_to_neighbor)
            {
                // if yes, update the distance to the current neighbor
                int updated_distance = distance_to_current_node + neighbor.edge_weight();
                result.distances.at(neighbor.id()) = updated_distance;
                result.predecessors.at(neighbor.id()) = current_node;
                result.predecessor_weights.at(neighbor.id()) = neighbor.edge_weight();

                // queue the neighbor
                // possibly it is already queued with a larger key,
                // but we have ensured above that no vertex is visited twice
                dijkstra_queue.push(std::make_pair(neighbor.id(), updated_distance));
            }
        }
    }

    return result;
}

/**
 * computes the metric closure of a graph
 * stores the resulting distances in distance_matrix
 * such that distance_matrix.at(i).at(j) is the distance of nodes i and j
 * stores the predecessors on the corresponding paths in predecessor_matrix
 * such that predecessor_matrix.at(i).at(j) is the predecessor
 * of j on the path from i to j
 * stores the corresponding edge weights in the path as well
 */
SteinerGraph::MetricClosureStruct SteinerGraph::metric_closure()
    const
{
    // initialise the result matrices
    MetricClosureStruct result;
    result.distance_matrix = std::vector<std::vector<int>>();
    result.predecessor_matrix = std::vector<std::vector<std::optional<NodeId>>>();
    result.predecessor_weight_matrix = std::vector<std::vector<int>>();

    // perform Dijkstra from every node
    for (NodeId i = 0; i < num_nodes(); i++)
    {
        const DijkstraStruct dijkstra_result = dijkstra(i);

        // add the results to the result matrices
        result.distance_matrix.push_back(dijkstra_result.distances);                     // distances from node i
        result.predecessor_matrix.push_back(dijkstra_result.predecessors);               // predecessors on the path from i
        result.predecessor_weight_matrix.push_back(dijkstra_result.predecessor_weights); // weights of the corresponding edges
    }

    return result;
}

/**
 * converts a metric_closure_distance_matrix to a graph
 * @param metric_closure_distance_matrix metric closure distance matrix from metric_closure()
 */
SteinerGraph SteinerGraph::metric_closure_graph(
    std::vector<std::vector<int>> metric_closure_distance_matrix) const
{
    SteinerGraph result = clear_edges();
    for (NodeId i = 0; i < num_nodes(); i++)
    {
        for (NodeId j = i + 1; j < num_nodes(); j++)
        {
            int distance = metric_closure_distance_matrix.at(i).at(j);
            result.add_edge(i, j, distance);
        }
    }

    return result;
}

/**
 * returns a terminal node
 */
std::optional<SteinerGraph::NodeId> SteinerGraph::find_terminal_node() const
{
    if (_terminals.size() == 0)
    {
        return std::nullopt;
    }

    return _terminals.at(0);
}

/**
 * check whether graph is connected (i.e. no distance is infinite)
 * @param metric_closure_distance_matrix metric closure distance matrix from metric_closure()
 */
void SteinerGraph::check_connected_metric_closure(
    const std::vector<std::vector<int>> &metric_closure_distance_matrix)
    const
{
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

/**
 * computes all distances between nodes and checks whether the graph is connected
 * stores the result in _distance_matrix
 */
void SteinerGraph::compute_distances_and_check_connected()
{
    // check if the distances have already been computed
    if (_computed_distance_matrix)
    {
        return;
    }

    const MetricClosureStruct metric_closure_result = metric_closure();
    _distance_matrix = metric_closure_result.distance_matrix;
    check_connected_metric_closure(_distance_matrix);
    _computed_distance_matrix = true;
}

/**
 * returns the distance between two nodes dynamically
 */
int SteinerGraph::get_or_compute_distance(
    const NodeId node1, const NodeId node2)
{
    compute_distances_and_check_connected();
    return _distance_matrix.at(node1).at(node2);
}