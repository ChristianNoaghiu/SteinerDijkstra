#include "steinergraph.h"
#include "dijkstra_steiner.h"
#include <bitset>

double DijkstraSteiner::bound(const bool lower_bound, const SteinerGraph::NodeId node, const DijkstraSteiner::TerminalSubset &R_without_I)
{
    double bound = 0;
    if (lower_bound)
    {
        /** @todo do this right */
        bound = get_or_compute_tsp_bound(node, R_without_I);
    }
    return bound;
}