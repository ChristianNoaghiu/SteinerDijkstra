#include "steinergraph.h"
#include <queue>

#include <bitset>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <vector>

// Hash und KeyEqual für die unordered_map labels
struct pair_hash
{ // Hashfunktion für die Paare (v, I) aus dem Definitionsbereich der label-Abbildung
    std::size_t operator()(const std::pair<SteinerGraph::NodeId, std::bitset<64>> &key) const
    {
        return (std::hash<SteinerGraph::NodeId>()(key.first) ^ std::hash<std::bitset<64>>()(key.second)); // XOR für Unterscheidung von (v, I) und (w, I) mit v != w
    }
};

struct pair_equal
{ // Operator der Gleichheit zweier Elemente der Form (v, I) prüft
    bool operator()(const std::pair<SteinerGraph::NodeId, std::bitset<64>> &first_elem, const std::pair<SteinerGraph::NodeId, std::bitset<64>> &second_elem) const
    {
        return (first_elem.first == first_elem.first && second_elem.second == second_elem.second);
    }
};
namespace
{
    std::bitset<64> minus_one(const std::bitset<64> &input)
    {
        std::bitset<64> output = input;
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
std::vector<std::pair<int, int>> SteinerGraph::dijkstra_steiner(NodeId r0, bool lower_bound)
{

    // check if r0 is a terminal
    if (_terminals.find(r0) == _terminals.end())
    {
        throw std::invalid_argument("r0 is not a terminal");
    }
    std::bitset<64> helper_variable = 0;
    // labels definition
    std::unordered_map<std::pair<SteinerGraph::NodeId, std::bitset<64>>, double> labels;
    // backtrack definition
    std::unordered_map<std::pair<SteinerGraph::NodeId, std::bitset<64>>, std::vector<std::pair<SteinerGraph::NodeId, std::bitset<64>>>> backtrack; // tuple von pairs, da mehrere label-pairs zu einem (v, I) gehören können
    // non_permanent_labels definition (N)
    std::priority_queue<std::pair<double, std::pair<SteinerGraph::NodeId, std::bitset<64>>>, std::vector<std::pair<double, std::pair<SteinerGraph::NodeId, std::bitset<64>>>>, std::greater<std::pair<double, std::pair<SteinerGraph::NodeId, std::bitset<64>>>>> non_permanent_labels;
    // permanent_labels definition (P)
    std::unordered_set<std::pair<SteinerGraph::NodeId, std::bitset<64>>, pair_hash, pair_equal> permanent_labels;
    for (NodeId terminal : _terminals)
    {
        if (terminal == r0)
        {
            continue;
        }
        labels[std::make_pair(terminal, std::bitset<64>(1 << terminal))] = 0;
        non_permanent_labels.push(std::make_pair(0, std::make_pair(terminal, std::bitset<64>(1 << terminal))));
        helper_variable.set(terminal);
    }
    const std::pair<SteinerGraph::NodeId, std::bitset<64>> final_permanent_label = make_pair(r0, helper_variable);
    while (not permanent_labels.count(final_permanent_label))
    {
        std::pair<SteinerGraph::NodeId, std::bitset<64>> current_label = non_permanent_labels.top().second; // (v, I) des priority-queue-Elements (geordnet nach niedrigster Distanz)
        non_permanent_labels.pop();
        permanent_labels.insert(current_label);
        double current_label_value = labels[current_label];
        NodeId current_node = current_label.first;
        for (const auto &edge_node : get_node(current_node).adjacent_nodes()) // Iterieren über die von v ausgehenden Kanten
        {
            NodeId neighbour = edge_node.id();                                                                                  // zweiter Knoten der betrachteten Kante
            double edge_weight = edge_node.edge_weight();                                                                       // Kantengewicht
            std::pair<SteinerGraph::NodeId, std::bitset<64>> neighbour_label = std::make_pair(neighbour, current_label.second); // (w, I) für den Kantennachbarn
            if (not permanent_labels.count(neighbour_label))
            {
                if (not labels.count(neighbour_label) || current_label_value + edge_weight < labels[neighbour_label]) // Der erste Fall ist, falls der Nachbar noch keine Distanz bekommen hat, also unendlich weit weg ist, falls dies der Fall ist, ist das Statement true und wir kommen sofort in den if-bracket
                {
                    labels[neighbour_label] = current_label_value + edge_weight;
                    backtrack[neighbour_label] = {std::make_pair(current_label.first, current_label.second)};
                    std::bitset<64> bound_input_1 = helper_variable ^ current_label.second;
                    bound_input_1.set(r0);
                    non_permanent_labels.push(std::make_pair((current_label_value + edge_weight + bound(lower_bound, current_node, bound_input_1)), neighbour_label)); // Alle Elemente aus non_permanent_labels haben die Distanz !inklusive lower_bound-Wert! als Vergleichswert
                }
            }
        }

        std::bitset<64> R_r0_without_I = helper_variable ^ current_label.second;
        for (std::bitset<64> J = R_r0_without_I; J.any(); J = minus_one(J) & R_r0_without_I) // Iterieren über alle nicht-leeren Teilmengen J von (R \ {r_0}) \ I
        {
            std::pair<SteinerGraph::NodeId, std::bitset<64>> v_J_label = std::make_pair(current_label.first, J);
            if (permanent_labels.count(v_J_label)) // prüft ob (v, J) \elem P
            {
                std::pair<SteinerGraph::NodeId, std::bitset<64>> union_label = std::make_pair(current_label.first, J | current_label.second);
                if (not permanent_labels.count(union_label)) // prüft ob (v, J u I) \notelem P
                {
                    //  müssen prüfen ob label(v, I) + label(v, J) < label(v, I u J) ------- ist schon klar, das v, I und v, J in labels sind? Yes, (v, I) weil es in N gelandet ist und (v, J) weil es in P gelandet ist
                    if (not permanent_labels.count(union_label) || current_label_value + labels[v_J_label] < labels[union_label]) // Fall das (v, J u I) noch keine Distanz bekommen hat sind wir schon im true-Fall und wir kommen aufgrund des ||'s in das if-bracket, wo es auf einen Wert gesetzt wird
                    {
                        labels[union_label] = current_label_value + labels[v_J_label];
                        backtrack[union_label] = {current_label, v_J_label};
                        std::bitset<64> bound_input_2 = helper_variable ^ union_label.second;
                        bound_input_2.set(r0);
                        non_permanent_labels.push(std::make_pair(current_label_value + labels[v_J_label] + bound(lower_bound, current_node, bound_input_2), union_label));
                    }
                }
            }
        }
    }
    return {}; // hier fehlt noch backtrack von (r0, R\r0) zurückgeben und der Code dazu
}
