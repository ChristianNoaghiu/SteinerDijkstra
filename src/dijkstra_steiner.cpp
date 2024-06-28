#include "steinergraph.h"
#include "dijkstra_steiner.h"
#include <queue>

#include <bitset>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <vector>

DijkstraSteiner::DijkstraSteiner(const SteinerGraph &graph) : _graph(graph)
{
    for (SteinerGraph::TerminalId terminal = 0; terminal < graph.num_terminals(); terminal++)
    {
        _all_terminals.set(terminal);
    }
    // check if graph is connected
    const SteinerGraph::MetricClosureStruct metric_closure_result = _graph.metric_closure();
    const std::vector<std::vector<int>> &distance_matrix = metric_closure_result.distance_matrix;
    _graph.check_connected_metric_closure(distance_matrix);
}

SteinerGraph DijkstraSteiner::compute_optimal_steiner_tree(const SteinerGraph::NodeId r0, const bool lower_bound_bool)
{
    if (lower_bound_bool)
    {
        // initialize terminal_subsets_of_size
        // it is needed for the bound computation
        // terminal_subsets_of_size.at(n) stores all terminal subsets of size n
        _terminal_subsets_of_size.resize(_graph.num_terminals() + 1);
        for (TerminalSubset terminal_subset_runner = _all_terminals; terminal_subset_runner.any(); terminal_subset_runner = minus_one(terminal_subset_runner)) //  Die "Lauf-variable" läuft alle subsets ab
        {
            _terminal_subsets_of_size.at(terminal_subset_runner.count()).push_back(terminal_subset_runner);
        }
    }
    return compute_optimal_steiner_tree(_graph, r0, lower_bound_bool);
}

SteinerGraph DijkstraSteiner::compute_optimal_steiner_tree(const SteinerGraph &graph, const SteinerGraph::NodeId r0, const bool lower_bound_bool)
{
    return dijkstra_steiner_algorithm(graph, r0, lower_bound_bool, _all_terminals);
}

/**
 * computes a optimal Steiner tree on a given graph using the Dijkstra-Steiner algorithm
 */
SteinerGraph DijkstraSteiner::dijkstra_steiner_algorithm(
    const SteinerGraph &graph,
    const SteinerGraph::NodeId r0,
    const bool lower_bound_bool,
    const DijkstraSteiner::TerminalSubset &terminalsubset) // terminalsubset is a bitset, we need this for the bound using this algorithm on a terminal-subset
{

    // check if r0 is in the given terminal subset and a terminal
    if (!graph.get_node(r0).is_terminal())
    {
        throw std::invalid_argument("r0 is not a terminal");
    }
    // The following check abuses, that is_terminal_subset_of only checks the subset-relation for the bits 0, 1, ..., num_terminals()-1
    if (!is_terminal_subset_of(terminalsubset, TerminalSubset().set()))
    {
        throw std::invalid_argument("The given terminal subset is not a subset of the terminals of the graph");
    }
    SteinerGraph::TerminalId r0_terminal_id;
    TerminalSubset terminals_without_r0 = 0;
    // labels definition
    LabelKeyToIntMap labels;
    // backtrack definition
    LabelKeyToWeightedLabelKeyVectorMap backtrack_data; // vector of pairs, since several label-pairs can be associated with (v, I)
    // non_permanent_labels definition (N)
    std::priority_queue<WeightedLabelKey, std::vector<WeightedLabelKey>, CompareWeightedLabelKey> non_permanent_labels;
    // permanent_labels definition (P)
    LabelKeySet permanent_labels;

    // initialization
    for (SteinerGraph::TerminalId terminal_id = 0; terminal_id < graph.num_terminals(); terminal_id++)
    {
        const SteinerGraph::NodeId terminal_node_id = graph.get_terminals().at(terminal_id);
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
        non_permanent_labels.push(std::make_pair(0, terminal_label));
        terminals_without_r0.set(terminal_id);
    }
    // there is no need to initialize labelskeys with empty TerminalSubsets, since they won't appear in N (non_permanent_labels)

    if (!terminalsubset[r0_terminal_id])
    {
        throw std::invalid_argument("r0 is not in the terminalsubset");
    }

    TerminalSubset terminals_with_r0 = terminals_without_r0;
    terminals_with_r0.set(r0_terminal_id); // having this in the declaration as terminals_without_r0.set(...) would alter terminals_without_r0
    const LabelKey final_permanent_label = make_pair(r0, terminals_without_r0);
    // main loop doing all the work
    while (!permanent_labels.count(final_permanent_label))
    {
        if (non_permanent_labels.empty())
        {
            throw std::runtime_error("No path found");
        }
        LabelKey current_label = non_permanent_labels.top().second; // (v, I) of the priority-queue-element (pq ordered by lowest distance)
        non_permanent_labels.pop();
        permanent_labels.insert(current_label);
        double current_label_value = labels[current_label];
        const SteinerGraph::NodeId current_node = current_label.first;
        const TerminalSubset current_terminal_subset = current_label.second;
        for (const SteinerGraph::Neighbor &neighbor : graph.get_node(current_node).adjacent_nodes()) // Iterieren über die von v ausgehenden Kanten
        {
            const SteinerGraph::NodeId neighbor_id = neighbor.id();                               // second node of the edge
            const double edge_weight = neighbor.edge_weight();                                    // edge weight
            const LabelKey neighbor_label = std::make_pair(neighbor_id, current_terminal_subset); // (w, I) for the neighbor "w"
            if (!permanent_labels.count(neighbor_label))
            {
                if (!labels.count(neighbor_label) || current_label_value + edge_weight < labels[neighbor_label]) // If (w, I) doesn't have a labelvalue the "||" ensures that, since the first part of the statement is true, the second part is not evaluated
                {
                    labels[neighbor_label] = current_label_value + edge_weight;
                    backtrack_data[neighbor_label] = {std::make_pair(edge_weight, current_label)};
                    TerminalSubset bound_input_1 = terminals_with_r0 ^ current_terminal_subset;
                    non_permanent_labels.push(std::make_pair((current_label_value + edge_weight + bound(r0, lower_bound_bool, current_node, bound_input_1)), neighbor_label)); // Alle Elemente aus non_permanent_labels haben die Distanz !inklusive lower_bound-Wert! als Vergleichswert
                }
            }
        }

        TerminalSubset R_r0_without_I = terminals_without_r0 ^ current_terminal_subset;
        for (TerminalSubset J = R_r0_without_I; J.any(); J = minus_one(J) & R_r0_without_I) // Iterieren through all non-empty subsets J of (R \ {r_0}) \ I
        {
            LabelKey v_J_label_key = std::make_pair(current_node, J);
            if (permanent_labels.count(v_J_label_key)) // prüft ob (v, J) \elem P
            {
                LabelKey union_label_key = std::make_pair(current_node, J | current_terminal_subset);
                if (!permanent_labels.count(union_label_key)) // prüft ob (v, J u I) \notelem P
                {
                    // it is known, that there is a labelvalue for (v, I) and (v, J), it is not necessarily given for (v, J u I)
                    if (!labels.count(union_label_key) || current_label_value + labels[v_J_label_key] < labels[union_label_key]) // If (v, J u I)  doesn't have a labelvalue the "||" ensures that, since the first part of the statement is true, the second part is not evaluated
                    {
                        labels[union_label_key] = current_label_value + labels[v_J_label_key];
                        backtrack_data[union_label_key] = {std::make_pair(SteinerGraph::infinite_distance, current_label), std::make_pair(SteinerGraph::infinite_distance, v_J_label_key)};
                        TerminalSubset bound_input_2 = terminals_with_r0 ^ union_label_key.second;
                        non_permanent_labels.push(std::make_pair(current_label_value + labels[v_J_label_key] + bound(r0, lower_bound_bool, current_node, bound_input_2), union_label_key));
                    }
                }
            }
        }
    }
    std::vector<EdgeTuple> edge_vector = backtrack(backtrack_data, final_permanent_label);
    SteinerGraph result_graph = graph.clear_edges();
    for (EdgeTuple &edge : edge_vector)
    {
        result_graph.add_edge(std::get<0>(edge), std::get<1>(edge), std::get<2>(edge));
    }
    return result_graph;
}

std::vector<DijkstraSteiner::EdgeTuple> DijkstraSteiner::backtrack(
    const LabelKeyToWeightedLabelKeyVectorMap &backtrack_data,
    const LabelKey &current_label)
    const
{
    std::vector<EdgeTuple> result;
    if (backtrack_data.count(current_label) == 0)
    {
        return result;
    }
    const std::vector<WeightedLabelKey> predecessors = backtrack_data.at(current_label);
    if (predecessors.size() == 1)
    {
        const std::vector<EdgeTuple> temp = backtrack(backtrack_data, std::make_pair(predecessors.at(0).second.first, current_label.second));
        result.insert(result.end(), temp.begin(), temp.end());
        const int edge_weight = predecessors.at(0).first;
        result.push_back(std::make_tuple(predecessors.at(0).second.first, current_label.first, edge_weight));
        return result;
    }
    else
    {
        for (const WeightedLabelKey &predecessor_label : predecessors)
        {
            const std::vector<EdgeTuple> temp = backtrack(backtrack_data, predecessor_label.second);
            result.insert(result.end(), temp.begin(), temp.end());
        }
        return result;
    }
}