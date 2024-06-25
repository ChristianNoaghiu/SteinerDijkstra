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

namespace
{
    DijkstraSteiner::TerminalSubset minus_one(const DijkstraSteiner::TerminalSubset &input)
    {
        DijkstraSteiner::TerminalSubset output = input;
        unsigned int i = 0;
        while (output[i] == 0)
        {
            output.set(i);
            i++;
        }
        output.reset(i);
        return output;
    }
}

SteinerGraph DijkstraSteiner::compute_optimal_steiner_tree(const SteinerGraph::NodeId r0, const bool lower_bound)
{

    // check if r0 is a terminal
    if (!_graph.get_node(r0).is_terminal())
    {
        throw std::invalid_argument("r0 is not a terminal");
    }
    TerminalSubset helper_variable = 0;
    // labels definition
    LabelKeyToDoubleMap labels;
    // backtrack definition
    LabelKeyToWeightedLabelKeyVectorMap backtrack_data; // vector von pairs, da mehrere label-pairs zu einem (v, I) gehören können
    // non_permanent_labels definition (N)
    std::priority_queue<WeightedLabelKey, std::vector<WeightedLabelKey>, CompareWeightedLabelKey> non_permanent_labels;
    // permanent_labels definition (P)
    LabelKeySet permanent_labels;
    for (const SteinerGraph::NodeId &terminal : _graph.get_terminals())
    {
        if (terminal == r0)
        {
            continue;
        }
        LabelKey terminal_label = std::make_pair(terminal, TerminalSubset(1 << terminal));
        labels[terminal_label] = 0;
        non_permanent_labels.push(std::make_pair(0, terminal_label));
        helper_variable.set(terminal);
    }
    const LabelKey final_permanent_label = make_pair(r0, helper_variable);
    while (not permanent_labels.count(final_permanent_label))
    {
        LabelKey current_label = non_permanent_labels.top().second; // (v, I) des priority-queue-Elements (geordnet nach niedrigster Distanz)
        non_permanent_labels.pop();
        permanent_labels.insert(current_label);
        double current_label_value = labels[current_label];
        SteinerGraph::NodeId current_node = current_label.first;
        for (const SteinerGraph::Neighbor &neighbor : _graph.get_node(current_node).adjacent_nodes()) // Iterieren über die von v ausgehenden Kanten
        {
            SteinerGraph::NodeId neighbor_id = neighbor.id();                            // zweiter Knoten der betrachteten Kante
            double edge_weight = neighbor.edge_weight();                                 // Kantengewicht
            LabelKey neighbor_label = std::make_pair(neighbor_id, current_label.second); // (w, I) für den Kantennachbarn
            if (not permanent_labels.count(neighbor_label))
            {
                if (not labels.count(neighbor_label) || current_label_value + edge_weight < labels[neighbor_label]) // Der erste Fall ist, falls der Nachbar noch keine Distanz bekommen hat, also unendlich weit weg ist, falls dies der Fall ist, ist das Statement true und wir kommen sofort in den if-bracket
                {
                    labels[neighbor_label] = current_label_value + edge_weight;
                    backtrack_data[neighbor_label] = {std::make_pair(edge_weight, current_label)};
                    TerminalSubset bound_input_1 = helper_variable ^ current_label.second;
                    bound_input_1.set(r0);
                    non_permanent_labels.push(std::make_pair((current_label_value + edge_weight + bound(lower_bound, current_node, bound_input_1)), neighbor_label)); // Alle Elemente aus non_permanent_labels haben die Distanz !inklusive lower_bound-Wert! als Vergleichswert
                }
            }
        }

        TerminalSubset R_r0_without_I = helper_variable ^ current_label.second;
        for (TerminalSubset J = R_r0_without_I; J.any(); J = minus_one(J) & R_r0_without_I) // Iterieren über alle nicht-leeren Teilmengen J von (R \ {r_0}) \ I
        {
            LabelKey v_J_label = std::make_pair(current_label.first, J);
            if (permanent_labels.count(v_J_label)) // prüft ob (v, J) \elem P
            {
                LabelKey union_label = std::make_pair(current_label.first, J | current_label.second);
                if (not permanent_labels.count(union_label)) // prüft ob (v, J u I) \notelem P
                {
                    //  müssen prüfen ob label(v, I) + label(v, J) < label(v, I u J) ------- ist schon klar, das v, I und v, J in labels sind? Yes, (v, I) weil es in N gelandet ist und (v, J) weil es in P gelandet ist
                    if (not labels.count(union_label) || current_label_value + labels[v_J_label] < labels[union_label]) // Fall das (v, J u I) noch keine Distanz bekommen hat sind wir schon im true-Fall und wir kommen aufgrund des ||'s in das if-bracket, wo es auf einen Wert gesetzt wird
                    {
                        labels[union_label] = current_label_value + labels[v_J_label];
                        backtrack_data[union_label] = {std::make_pair(SteinerGraph::infinite_distance, current_label), std::make_pair(SteinerGraph::infinite_distance, v_J_label)};
                        TerminalSubset bound_input_2 = helper_variable ^ union_label.second;
                        bound_input_2.set(r0);
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