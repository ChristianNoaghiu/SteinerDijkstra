#include "steinergraph.h"
#include <bitset>

double SteinerGraph::bound(const bool lower_bound, const NodeId node, const TerminalSubset &R_without_I)
{
    double bound = 0;
    if (lower_bound)
    {
        /** @todo do this right */
        bound = get_or_compute_tsp_bound(node, R_without_I);
    }
    return bound;
}