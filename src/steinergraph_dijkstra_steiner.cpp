#include "steinergraph.h"
#include <queue>

#include <bitset>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <vector>

namespace
{
    SteinerGraph::TerminalSubset minus_one(const SteinerGraph::TerminalSubset &input)
    {
        SteinerGraph::TerminalSubset output = input;
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

std::vector<std::pair<int, int>> SteinerGraph::dijkstra_steiner(const NodeId r0, const bool lower_bound)
{

    // check if r0 is a terminal
    if (_terminals.find(r0) == _terminals.end())
    {
        throw std::invalid_argument("r0 is not a terminal");
    }
    TerminalSubset helper_variable = 0;
    // labels definition
    BoundKeyToDoubleMap labels;
    // backtrack definition
    BoundKeyToBoundKeyVectorMap backtrack; // tuple von pairs, da mehrere label-pairs zu einem (v, I) gehören können
    // non_permanent_labels definition (N)
    std::priority_queue<std::pair<double, BoundKey>, std::vector<std::pair<double, std::pair<SteinerGraph::NodeId, std::bitset<64>>>>, std::greater<std::pair<double, BoundKey>>> non_permanent_labels;
    // permanent_labels definition (P)
    BoundKeySet permanent_labels;
    for (NodeId terminal : _terminals)
    {
        if (terminal == r0)
        {
            continue;
        }
        labels[std::make_pair(terminal, TerminalSubset(1 << terminal))] = 0;
        non_permanent_labels.push(std::make_pair(0, std::make_pair(terminal, TerminalSubset(1 << terminal))));
        helper_variable.set(terminal);
    }
    const BoundKey final_permanent_label = make_pair(r0, helper_variable);
    while (not permanent_labels.count(final_permanent_label))
    {
        BoundKey current_label = non_permanent_labels.top().second; // (v, I) des priority-queue-Elements (geordnet nach niedrigster Distanz)
        non_permanent_labels.pop();
        permanent_labels.insert(current_label);
        double current_label_value = labels[current_label];
        NodeId current_node = current_label.first;
        for (const auto &edge_node : get_node(current_node).adjacent_nodes()) // Iterieren über die von v ausgehenden Kanten
        {
            NodeId neighbour = edge_node.id();                                          // zweiter Knoten der betrachteten Kante
            double edge_weight = edge_node.edge_weight();                               // Kantengewicht
            BoundKey neighbour_label = std::make_pair(neighbour, current_label.second); // (w, I) für den Kantennachbarn
            if (not permanent_labels.count(neighbour_label))
            {
                if (not labels.count(neighbour_label) || current_label_value + edge_weight < labels[neighbour_label]) // Der erste Fall ist, falls der Nachbar noch keine Distanz bekommen hat, also unendlich weit weg ist, falls dies der Fall ist, ist das Statement true und wir kommen sofort in den if-bracket
                {
                    labels[neighbour_label] = current_label_value + edge_weight;
                    backtrack[neighbour_label] = {current_label};
                    TerminalSubset bound_input_1 = helper_variable ^ current_label.second;
                    bound_input_1.set(r0);
                    non_permanent_labels.push(std::make_pair((current_label_value + edge_weight + bound(lower_bound, current_node, bound_input_1)), neighbour_label)); // Alle Elemente aus non_permanent_labels haben die Distanz !inklusive lower_bound-Wert! als Vergleichswert
                }
            }
        }

        TerminalSubset R_r0_without_I = helper_variable ^ current_label.second;
        for (TerminalSubset J = R_r0_without_I; J.any(); J = minus_one(J) & R_r0_without_I) // Iterieren über alle nicht-leeren Teilmengen J von (R \ {r_0}) \ I
        {
            BoundKey v_J_label = std::make_pair(current_label.first, J);
            if (permanent_labels.count(v_J_label)) // prüft ob (v, J) \elem P
            {
                BoundKey union_label = std::make_pair(current_label.first, J | current_label.second);
                if (not permanent_labels.count(union_label)) // prüft ob (v, J u I) \notelem P
                {
                    //  müssen prüfen ob label(v, I) + label(v, J) < label(v, I u J) ------- ist schon klar, das v, I und v, J in labels sind? Yes, (v, I) weil es in N gelandet ist und (v, J) weil es in P gelandet ist
                    if (not labels.count(union_label) || current_label_value + labels[v_J_label] < labels[union_label]) // Fall das (v, J u I) noch keine Distanz bekommen hat sind wir schon im true-Fall und wir kommen aufgrund des ||'s in das if-bracket, wo es auf einen Wert gesetzt wird
                    {
                        labels[union_label] = current_label_value + labels[v_J_label];
                        backtrack[union_label] = {current_label, v_J_label};
                        TerminalSubset bound_input_2 = helper_variable ^ union_label.second;
                        bound_input_2.set(r0);
                        non_permanent_labels.push(std::make_pair(current_label_value + labels[v_J_label] + bound(lower_bound, current_node, bound_input_2), union_label));
                    }
                }
            }
        }
    }
    return {}; // hier fehlt noch backtrack von (r0, R\r0) zurückgeben und der Code dazu
}
