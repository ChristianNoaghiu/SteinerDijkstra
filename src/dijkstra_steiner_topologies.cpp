#include "dijkstra_steiner.h"
#include <vector>
#include <queue>
#include <optional>

double DijkstraSteiner::compute_compare_bound(
    DijkstraSteiner::LabelKeyToIntMap &labels,
    const SteinerGraph::NodeId node,
    const DijkstraSteiner::TerminalSubset &terminal_subset,
    const SteinerGraph::NodeId r0)
{
    const LabelKey label_key = std::make_pair(node, terminal_subset);
    if (!labels.count(label_key))
    {
        return std::numeric_limits<double>::infinity();
    }

    double label = labels[label_key];
    double complement_bound = bound(r0, true, node, _all_terminals ^ terminal_subset);
    return label + complement_bound;
}

void DijkstraSteiner::test_get_topologies()
{
    std::cout << get_or_compute_distance(0, 1) << std::endl;
    DijkstraSteiner::TerminalSubset terminal_subset = 0b111;
    std::vector<DijkstraSteiner::TopologyStruct> topologies = get_topologies(0, terminal_subset, 2);
    for (const DijkstraSteiner::TopologyStruct &topology : topologies)
    {
        topology.topology.print();
    }
}

std::vector<DijkstraSteiner::TopologyStruct> DijkstraSteiner::get_topologies(
    const SteinerGraph::NodeId r0,
    const DijkstraSteiner::TerminalSubset &terminalsubset,
    const int max_detour)
{
    // check if r0 is in the given terminal subset and a terminal
    if (!_graph.get_node(r0).is_terminal())
    {
        throw std::invalid_argument("r0 is not a terminal");
    }

    SteinerGraph::TerminalId r0_terminal_id = -1;
    TerminalSubset terminals_without_r0 = 0;
    // labels definition
    LabelKeyToIntMap labels;
    // backtrack definition
    DetourLabelKeyToLabelKeyVectorVectorMap backtrack_data;
    // permanent_labels definition (P)
    LabelKeySet permanent_labels;

    // initialization
    for (SteinerGraph::TerminalId terminal_id = 0; terminal_id < _graph.num_terminals(); terminal_id++)
    {
        const SteinerGraph::NodeId terminal_node_id = _graph.get_terminals().at(terminal_id);
        if (terminal_node_id == r0)
        {
            r0_terminal_id = terminal_id;
            continue;
        }
        if (!terminalsubset[terminal_id])
        {
            continue;
        }
        const LabelKey terminal_label = std::make_pair(terminal_node_id, (TerminalSubset(1) << terminal_id));
        labels[terminal_label] = 0;
        terminals_without_r0.set(terminal_id);
    }

    if (!terminalsubset[r0_terminal_id])
    {
        throw std::invalid_argument("r0 is not in the terminalsubset");
    }

    std::priority_queue<WeightedLabelKey, std::vector<WeightedLabelKey>, CompareWeightedLabelKey> non_permanent_labels;

    // insert all (v, I) with I nonempty and not containing r0 into the priority queue
    for (SteinerGraph::NodeId node = 0; node < _graph.num_nodes(); node++)
    {
        for (TerminalSubset terminal_subset = _all_terminals; terminal_subset.any(); terminal_subset = minus_one(terminal_subset))
        {
            if (terminal_subset[r0_terminal_id] == 1)
            {
                continue;
            }
            const LabelKey label = std::make_pair(node, terminal_subset);
            const double bound = compute_compare_bound(labels, node, terminal_subset, r0);
            non_permanent_labels.push(std::make_pair(bound, label));
        }
    }

    double optimum = std::numeric_limits<double>::infinity();

    do
    {
        LabelKey current_label = non_permanent_labels.top().second; // (v, I) of the priority-queue-element (pq ordered by lowest distance)
        non_permanent_labels.pop();
        if (permanent_labels.count(current_label))
        {
            continue;
        }

        permanent_labels.insert(current_label);
        double current_label_value = labels[current_label];
        const SteinerGraph::NodeId current_node = current_label.first;
        const TerminalSubset current_terminal_subset = current_label.second;

        for (const SteinerGraph::Neighbor &neighbor : _graph.get_node(current_node).adjacent_nodes()) // Iterieren über die von v ausgehenden Kanten
        {
            const SteinerGraph::NodeId neighbor_id = neighbor.id();                               // second node of the edge
            const double edge_weight = neighbor.edge_weight();                                    // edge weight
            const LabelKey neighbor_label = std::make_pair(neighbor_id, current_terminal_subset); // (w, I) for the neighbor "w"
            if (!permanent_labels.count(neighbor_label))
            {
                if (!labels.count(neighbor_label) || current_label_value + edge_weight < labels[neighbor_label]) // If (w, I) doesn't have a labelvalue the "||" ensures that, since the first part of the statement is true, the second part is not evaluated
                {
                    labels[neighbor_label] = current_label_value + edge_weight;
                    const double new_compare_bound = compute_compare_bound(labels, neighbor_id, current_terminal_subset, r0);
                    non_permanent_labels.push(std::make_pair(new_compare_bound, neighbor_label)); // Alle Elemente aus non_permanent_labels haben die Distanz !inklusive lower_bound-Wert! als Vergleichswert
                }
            }
        }

        TerminalSubset R_r0_without_I = terminals_without_r0 ^ current_terminal_subset;
        for (TerminalSubset J = R_r0_without_I; J.any(); J = minus_one(J) & R_r0_without_I) // Iterieren through all non-empty subsets J of (R \ {r_0}) \ I
        {
            LabelKey v_J_label = std::make_pair(current_node, J);
            if (labels[v_J_label] != std::numeric_limits<double>::max()) // prüft ob (v, J) \elem P
            {
                LabelKey union_label = std::make_pair(current_node, J | current_terminal_subset);
                // it is known, that there is a labelvalue for (v, I) and (v, J), it is not necessarily given for (v, J u I)
                if (!labels.count(union_label) || current_label_value + labels[v_J_label] < labels[union_label]) // If (v, J u I)  doesn't have a labelvalue the "||" ensures that, since the first part of the statement is true, the second part is not evaluated
                {
                    labels[union_label] = current_label_value + labels[v_J_label];
                    const double new_compare_bound = compute_compare_bound(labels, current_node, current_terminal_subset, r0);
                    non_permanent_labels.push(std::make_pair(new_compare_bound, union_label));
                }
            }
        }

        if (current_node == r0 && current_terminal_subset == terminals_without_r0)
        {
            optimum = current_label_value;
        }

        if (compute_compare_bound(labels, current_node, current_terminal_subset, r0) > optimum + max_detour)
        {
            break;
        }
    } while (non_permanent_labels.size() > 0);

    for (const LabelKey &label : permanent_labels)
    {
        const SteinerGraph::NodeId &current_node = label.first;
        const TerminalSubset &current_terminal_subset = label.second;
        if (current_terminal_subset.count() == 0)
        {
            continue;
        }

        for (const SteinerGraph::Neighbor &neighbor : _graph.get_node(current_node).adjacent_nodes())
        {
            const SteinerGraph::NodeId neighbor_id = neighbor.id(); // second node of the edge
            const double edge_weight = neighbor.edge_weight();      // edge weight
            const LabelKey neighbor_label = std::make_pair(neighbor_id, current_terminal_subset);
            double zeta = labels[neighbor_label] + edge_weight - labels[label];

            if (zeta <= max_detour)
            {
                DetourLabelKey detour_label = std::make_tuple(zeta, current_node, current_terminal_subset);
                backtrack_data[detour_label].push_back({neighbor_label});
            }
        }

        // iterate over all nonempety J that are proper subsets of I
        for (TerminalSubset terminal_subset_j = current_terminal_subset; terminal_subset_j.any(); terminal_subset_j = minus_one(terminal_subset_j) & current_terminal_subset)
        {
            if (terminal_subset_j == current_terminal_subset)
            {
                continue;
            }

            LabelKey v_J_label = std::make_pair(current_node, terminal_subset_j);
            if (labels[v_J_label] == std::numeric_limits<double>::max())
            {
                continue;
            }

            const TerminalSubset current_terminal_subset_without_J = current_terminal_subset ^ terminal_subset_j;
            const LabelKey without_J_label = std::make_pair(current_node, current_terminal_subset_without_J);
            const double zeta = labels[v_J_label] + labels[without_J_label] - labels[label];
            if (zeta <= max_detour)
            {
                DetourLabelKey detour_label = std::make_tuple(zeta, current_node, current_terminal_subset);
                backtrack_data[detour_label].push_back({v_J_label, without_J_label});
            }
        }
    }

    return enumerate_topologies(r0, terminals_without_r0, max_detour, max_detour, backtrack_data);
}

/**
 * returns the enumeration of topologies for a terminal subset containing exactly one terminal
 */
std::vector<DijkstraSteiner::TopologyStruct> DijkstraSteiner::enumerate_topologies_one_element_terminal_subset(
    const SteinerGraph::NodeId node,
    const TerminalSubset &terminal_subset)
{
    if (terminal_subset.count() != 1)
    {
        throw std::invalid_argument("Terminal subset must contain exactly one terminal");
    }

    for (SteinerGraph::TerminalId terminal_id = 0; terminal_id < _graph.num_terminals(); terminal_id++)
    {
        if (!terminal_subset[terminal_id])
        {
            continue;
        }

        const SteinerGraph::NodeId terminal_node = _graph.get_terminals().at(terminal_id);

        SteinerGraph result_graph = _graph.clear_edges();
        std::vector<bool> existent_nodes(_graph.num_nodes(), false);
        std::vector<EdgeTuple> existent_edges;

        if (terminal_node == node)
        {
            existent_nodes[terminal_node] = true;
        }
        else
        {
            result_graph.add_edge(node, terminal_node);
            existent_nodes[node] = true;
            existent_nodes[terminal_node] = true;
            existent_edges.push_back({node, terminal_node, get_or_compute_distance(node, terminal_node)});
        }

        const TopologyStruct topology = {result_graph, existent_nodes, existent_edges, 0};
        return {topology};
    }

    throw std::invalid_argument("Invalid terminal_subset");
}

/** @todo remove this */
void DijkstraSteiner::test_enumerate_topologies_one_element_terminal_subset()
{
    auto topologies = enumerate_topologies_one_element_terminal_subset(3, 0b010);
    std::cout << "Enumerate topologies for one element terminal subset" << std::endl;
}

SteinerGraph DijkstraSteiner::compute_backtracking_graph(
    const SteinerGraph::NodeId node,
    const TerminalSubset &terminal_subset,
    const int max_detour,
    DetourLabelKeyToLabelKeyVectorVectorMap &backtrack_data)
{
    SteinerGraph backtracking_graph = _graph.clear_edges();
    backtracking_graph.make_directed();

    // adjacency matrix to avoid parallel edges
    std::vector<std::vector<bool>> adjacency_matrix(_graph.num_nodes(), std::vector<bool>(_graph.num_nodes(), false));

    for (int detour = 0; detour <= max_detour; detour++)
    {
        DetourLabelKey backtrack_key = std::make_tuple(detour, node, terminal_subset);
        for (std::vector<LabelKey> &backtrack_element : backtrack_data[backtrack_key])
        {
            if (backtrack_element.size() != 1)
            {
                continue;
            }

            const LabelKey &neighbor_label = backtrack_element[0];
            const SteinerGraph::NodeId neighbor_node = neighbor_label.first;
            const int edge_weight = detour;

            if (adjacency_matrix.at(node).at(neighbor_node))
            {
                // if edge already exists, then it has been added for a smaller zeta,
                // so we can skip this edge
                continue;
            }

            backtracking_graph.add_edge(node, neighbor_node, edge_weight);
            adjacency_matrix.at(node).at(neighbor_node) = true;
        }
    }

    return backtracking_graph;
}

/**
 * checks if the given topology is a tree
 */
bool DijkstraSteiner::is_tree(const DijkstraSteiner::TopologyStruct &topology_struct) const
{
    const SteinerGraph &topology = topology_struct.topology;
    const std::vector<bool> &existent_nodes = topology_struct.existent_nodes;

    // find a start node (or throw an error if none exists)
    std::optional<SteinerGraph::NodeId> start_node_optional;
    for (SteinerGraph::NodeId node = 0; node < _graph.num_nodes(); node++)
    {
        if (!existent_nodes.at(node))
        {
            continue;
        }

        start_node_optional = node;
        break;
    }

    if (!start_node_optional.has_value())
    {
        throw std::invalid_argument("Topology is empty");
    }

    const SteinerGraph::NodeId start_node = start_node_optional.value();
    std::vector<bool> visited(topology.num_nodes(), false);
    std::vector<std::optional<SteinerGraph::NodeId>> predecessors(topology.num_nodes(), std::nullopt);
    std::queue<SteinerGraph::NodeId> queue;
    queue.push(start_node);

    unsigned int counted_edges = 0;

    // check if the topology is connected via BFS
    while (!queue.empty())
    {
        const SteinerGraph::NodeId current_node = queue.front();
        queue.pop();

        // check if current_node really exists
        if (!existent_nodes.at(current_node))
        {
            throw std::runtime_error("Topology contains non-existent node");
        }

        if (visited.at(current_node))
        {
            return false;
        }

        visited.at(current_node) = true;

        for (const SteinerGraph::Neighbor &neighbor : topology.get_node(current_node).adjacent_nodes())
        {
            const SteinerGraph::NodeId neighbor_id = neighbor.id();
            if (!existent_nodes.at(neighbor_id) || neighbor_id == predecessors.at(current_node))
            {
                continue;
            }

            // the graph contains a cycle if a node is visited twice
            if (visited.at(neighbor_id))
            {
                return false;
            }

            counted_edges++;
            queue.push(neighbor_id);
            predecessors.at(neighbor_id) = current_node;
        }
    }

    // check if all edges have been counted
    if (counted_edges != topology_struct.existent_edges.size())
    {
        return false;
    }
    // if yes, then the graph consists of exactly one connected component
    // which doesn't contain a cycle, therefore it is a tree

    return true;
}

/** @todo remove this */
void DijkstraSteiner::test_is_tree()
{
    SteinerGraph graph(4);
    graph.add_edge(0, 1);
    graph.add_edge(1, 2);

    std::vector<bool> existent_nodes = {true, true, true, false};
    std::vector<EdgeTuple> existent_edges = {{0, 1, 3}, {1, 2, 5}};
    DijkstraSteiner::TopologyStruct topology_struct = {graph, existent_nodes, existent_edges, 0};

    if (is_tree(topology_struct))
    {
        std::cout << "Topology is a tree" << std::endl;
    }
    else
    {
        std::cout << "Topology is not a tree" << std::endl;
    }
}

/**
 * enumerate_topologies as in the paper
 */
std::vector<DijkstraSteiner::TopologyStruct> DijkstraSteiner::enumerate_topologies(
    const SteinerGraph::NodeId node,
    const TerminalSubset &terminal_subset,
    const int zeta,
    const int max_detour,
    DetourLabelKeyToLabelKeyVectorVectorMap &backtrack_data)
{
    if (terminal_subset.count() == 1)
    {
        return enumerate_topologies_one_element_terminal_subset(node, terminal_subset);
    }

    SteinerGraph backtracking_graph = compute_backtracking_graph(node, terminal_subset, max_detour, backtrack_data);

    std::vector<std::pair<SteinerGraph::NodeId, int>> reachable_nodes;
    for (SteinerGraph::NodeId i = 0; i < backtracking_graph.num_nodes(); i++)
    {
        int distance_to_i = get_or_compute_distance(node, i);
        if (distance_to_i <= zeta)
        {
            reachable_nodes.push_back(std::make_pair(i, distance_to_i));
        }
    }

    std::vector<TopologyStruct> result;
    for (const std::pair<SteinerGraph::NodeId, int> &reachable_node : reachable_nodes)
    {
        const SteinerGraph::NodeId reachable_node_id = reachable_node.first;
        const SteinerGraph::NodeId reachable_node_zeta = reachable_node.second;
        for (int zeta_part = 0; zeta_part <= zeta - reachable_node_zeta; zeta_part++)
        {
            DetourLabelKey backtrack_key = std::make_tuple(zeta_part, node, terminal_subset);
            for (std::vector<LabelKey> &backtrack_element : backtrack_data[backtrack_key])
            {
                if (backtrack_element.size() != 2)
                {
                    continue;
                }

                const LabelKey &label1 = backtrack_element.at(0);
                const TerminalSubset terminal_subset1 = label1.second;
                const LabelKey &label2 = backtrack_element.at(1);
                const TerminalSubset terminal_subset2 = label2.second;

                int zeta_rem = zeta - reachable_node_zeta - zeta_part;
                std::vector<TopologyStruct> topologies1 = enumerate_topologies(reachable_node_id, terminal_subset1, zeta_rem, max_detour, backtrack_data);
                std::vector<TopologyStruct> topologies2 = enumerate_topologies(reachable_node_id, terminal_subset2, zeta_rem, max_detour, backtrack_data);

                for (TopologyStruct &topology1 : topologies1)
                {
                    for (TopologyStruct &topology2 : topologies2)
                    {
                        if (topology1.detour + topology2.detour > zeta_rem)
                        {
                            continue;
                        }

                        SteinerGraph new_topology = _graph.clear_edges();
                        std::vector<bool> existent_nodes(_graph.num_nodes(), false);
                        std::vector<EdgeTuple> existent_edges;
                        existent_nodes[node] = true;

                        for (SteinerGraph::NodeId node = 0; node < _graph.num_nodes(); node++)
                        {
                            if (topology1.existent_nodes.at(node) || topology2.existent_nodes.at(node))
                            {
                                existent_nodes[node] = true;
                            }
                        }

                        for (const EdgeTuple &edge : topology1.existent_edges)
                        {
                            new_topology.add_edge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
                            existent_edges.push_back(edge);
                        }

                        for (const EdgeTuple &edge : topology2.existent_edges)
                        {
                            new_topology.add_edge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
                            existent_edges.push_back(edge);
                        }

                        if (node != reachable_node_id)
                        {
                            new_topology.add_edge(node, reachable_node_id);
                            existent_edges.push_back({node, reachable_node_id, reachable_node_zeta});
                        }

                        const int new_zeta = reachable_node_zeta + zeta_part + topology1.detour + topology2.detour;
                        const TopologyStruct new_topology_struct = {new_topology, existent_nodes, existent_edges, new_zeta};

                        if (is_tree(new_topology_struct))
                        {
                            result.push_back(new_topology_struct);
                        }
                    }
                }
            }
        }
    }

    return result;
}