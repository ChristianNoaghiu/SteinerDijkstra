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
        return [](const NodeDistancePair &node_distance1,
                  const NodeDistancePair &node_distance2)
        {
            const int distance1 = node_distance1.second;
            const int distance2 = node_distance2.second;

            return (distance1 > distance2);
        };
    }

    std::function<bool(const SteinerGraph::NodeId)> is_in_graph()
    {
        return [](const SteinerGraph::NodeId node)
        {
            /**
             * @todo Wunused complains about node not being used,
             * therefore this redundant comparison
             */
            if (node + 1 == node + 1)
                return true;
            return true;
        };
    }
}

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
 * computes a MST the subgraph induced by is_in_subgraph
 * using Prim's algorithm (in the connected component of start_node)
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

    if (start_node >= num_nodes())
    {
        throw std::runtime_error("Invalid start_node");
    }

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

    return *_terminals.begin();
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
 * computes a MST on the terminal subgraph in the metric closure using Prim's algorithm
 * and stores the predecessors of a corresponding rooted arborescence in predecessors
 * @param metric_closure_distance_matrix metric closure distance matrix from metric_closure()
 */
std::vector<std::optional<SteinerGraph::NodeId>> SteinerGraph::terminal_rooted_mst_predecessors(
    const std::vector<std::vector<int>> &metric_closure_distance_matrix)
    const
{
    check_connected_metric_closure(metric_closure_distance_matrix);

    // pick a start node for Prim
    const std::optional<NodeId> start_node_optional = find_terminal_node();

    if (!start_node_optional.has_value())
    {
        throw std::runtime_error("No terminal node exists.");
    }

    const NodeId start_node = start_node_optional.value();

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

            if (edge_weight_to_neighbor < distance_to_neighbor)
            {
                // if yes, update the neighbor's distance
                distances.at(neighbor_id) = edge_weight_to_neighbor;
                predecessors.at(neighbor_id) = current_node;

                prim_queue.push(std::make_pair(neighbor_id, edge_weight_to_neighbor));
            }
        }
    }

    return predecessors;
}

/**
 * decodes the MST-metric-closure-path from a start_node into the real
 * path of the graph and adds all edges on the way to the result_graph
 * of the steiner_tree_mst_approximation
 */
void SteinerGraph::add_path_to_steiner_tree_mst_approximation(
    const SteinerGraph::NodeId &start_node,
    const std::vector<std::vector<std::optional<SteinerGraph::NodeId>>> &metric_closure_predecessor_matrix,
    const std::vector<std::vector<int>> &metric_closure_predecessor_weight_matrix,
    const std::vector<std::optional<SteinerGraph::NodeId>> &mst_predecessors,
    std::vector<bool> &visited,
    SteinerGraph &result_graph)
    const
{
    const std::optional<NodeId> mst_predecessor_optional = mst_predecessors.at(start_node);

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

    const std::vector<std::optional<NodeId>> mst_predecessors = terminal_rooted_mst_predecessors(metric_closure_result.distance_matrix);

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
            mst_predecessors,
            visited,
            result_graph);
    }

    // return a MST in the result graph
    // terminal_node must have a value since _terminals.size() == 0 has been excluded above
    const NodeId terminal_node = find_terminal_node().value();
    return result_graph.component_mst(terminal_node);
}