#include "steinergraph.h"
#include "dijkstra_steiner.h"
#include <bitset>

double DijkstraSteiner::bound(
    const SteinerGraph::NodeId r0,
    const bool lower_bound_bool,
    const SteinerGraph::NodeId node,
    const DijkstraSteiner::TerminalSubset &terminal_subset)
{
    double bound = 0;
    if (lower_bound_bool)
    {
        // const double j_terminal_bound = get_or_compute_j_terminal_bound(1, r0, node, terminal_subset);
        const double one_tree_bound = get_or_compute_one_tree_bound(node, terminal_subset, r0);
        // const double tsp_bound = get_or_compute_tsp_bound(node, terminal_subset);
        if (one_tree_bound < 0)
        {
            throw std::runtime_error("One-tree bound must be non-negative.");
        }
        bound = one_tree_bound;
    }
    return bound;
}