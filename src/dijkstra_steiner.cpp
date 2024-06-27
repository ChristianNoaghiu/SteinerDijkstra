#include "steinergraph.h"
#include "dijkstra_steiner.h"

DijkstraSteiner::DijkstraSteiner(const SteinerGraph &graph) : _graph(graph) {}

#include "steinergraph.h"
#include <queue>

#include <bitset>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <vector>

SteinerGraph DijkstraSteiner::compute_optimal_steiner_tree(const SteinerGraph::NodeId r0, const bool lower_bound)
{
    TerminalSubset terminals = 0;
    for (const SteinerGraph::NodeId &terminal : _graph.get_terminals())
    {
        terminals.set(terminal);
    }
    return dijkstra_steiner_algorithm(r0, lower_bound, terminals);
}

SteinerGraph DijkstraSteiner::dijkstra_steiner_algorithm(
    const SteinerGraph::NodeId r0,
    const bool lower_bound,
    const DijkstraSteiner::TerminalSubset &terminalsubset) // terminalsubset is a bitset, we need this for the bound using this algorithm on a terminal-subset
{

    // check if r0 is in the given terminal subset and a terminal
    if (terminalsubset[r0] && !_graph.get_node(r0).is_terminal())
    {
        throw std::invalid_argument("r0 is either not a terminal or not in the given terminal subset");
    }
    // The following check abuses, that is_terminal_subset_of only checks the subset-relation for the bits 0, 1, ..., num_terminals()-1
    if (!is_terminal_subset_of(terminalsubset, TerminalSubset().set()))
    {
        throw std::invalid_argument("The given terminal subset is not a subset of the terminals of the graph");
    }

    TerminalSubset terminals_without_r0 = 0; /** @todo const and immediately define after bitset change*/
    // labels definition
    LabelKeyToDoubleMap labels;
    // backtrack definition
    LabelKeyToWeightedLabelKeyVectorMap backtrack_data; // vector of pairs, since several label-pairs can be associated with (v, I)
    // non_permanent_labels definition (N)
    std::priority_queue<WeightedLabelKey, std::vector<WeightedLabelKey>, CompareWeightedLabelKey> non_permanent_labels;
    // permanent_labels definition (P)
    LabelKeySet permanent_labels;

    // initialization
    for (SteinerGraph::TerminalId terminal_id = 0; terminal_id < _graph.num_terminals(); terminal_id++)
    {
        const SteinerGraph::NodeId terminal_node_id = _graph.get_terminals().at(terminal_id);
        if (terminal_node_id == r0)
        {
            continue;
        }
        const LabelKey terminal_label = std::make_pair(terminal_node_id, (TerminalSubset(1) << terminal_id));
        labels[terminal_label] = 0;
        non_permanent_labels.push(std::make_pair(0, terminal_label));
        terminals_without_r0.set(terminal_id);
    }
    // no need to initialize labelskeys with empty TerminalSubsets, since they won't appear in N (non_permanent_labels)

    const LabelKey final_permanent_label = make_pair(r0, terminals_without_r0);
    // main loop doing all the work
    while (!permanent_labels.count(final_permanent_label))
    {
        LabelKey current_label = non_permanent_labels.top().second; // (v, I) of the priority-queue-element (pq ordered by lowest distance)
        non_permanent_labels.pop();
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
                    backtrack_data[neighbor_label] = {std::make_pair(edge_weight, current_label)};
                    TerminalSubset bound_input_1 = terminals_without_r0 ^ current_terminal_subset;
                    bound_input_1.set(r0);                                                                                                                            /** @todo const this after bitset change */
                    non_permanent_labels.push(std::make_pair((current_label_value + edge_weight + bound(lower_bound, current_node, bound_input_1)), neighbor_label)); // Alle Elemente aus non_permanent_labels haben die Distanz !inklusive lower_bound-Wert! als Vergleichswert
                }
            }
        }

        TerminalSubset R_r0_without_I = terminals_without_r0 ^ current_terminal_subset;
        for (TerminalSubset J = R_r0_without_I; J.any(); J = minus_one(J) & R_r0_without_I) // Iterieren through all non-empty subsets J of (R \ {r_0}) \ I
        {
            LabelKey v_J_label = std::make_pair(current_node, J);
            if (permanent_labels.count(v_J_label)) // prüft ob (v, J) \elem P
            {
                LabelKey union_label = std::make_pair(current_node, J | current_terminal_subset);
                if (!permanent_labels.count(union_label)) // prüft ob (v, J u I) \notelem P
                {
                    // it is known, that there is a labelvalue for (v, I) and (v, J), it is not necessarily given for (v, J u I)
                    if (!labels.count(union_label) || current_label_value + labels[v_J_label] < labels[union_label]) // If (v, J u I)  doesn't have a labelvalue the "||" ensures that, since the first part of the statement is true, the second part is not evaluated
                    {
                        labels[union_label] = current_label_value + labels[v_J_label];
                        backtrack_data[union_label] = {std::make_pair(SteinerGraph::infinite_distance, current_label), std::make_pair(SteinerGraph::infinite_distance, v_J_label)};
                        TerminalSubset bound_input_2 = terminals_without_r0 ^ union_label.second;
                        bound_input_2.set(r0); /** @todo const this after bitset change */
                        non_permanent_labels.push(std::make_pair(current_label_value + labels[v_J_label] + bound(lower_bound, current_node, bound_input_2), union_label));
                    }
                }
            }
        }
    }
    std::vector<EdgeTuple> edge_vector = backtrack(backtrack_data, final_permanent_label);
    SteinerGraph result_graph = _graph.clear_edges();
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