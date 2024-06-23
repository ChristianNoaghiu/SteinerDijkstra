#include "steinergraph.h"
#include <bitset>

double SteinerGraph::bound(bool lower_bound, NodeId node, std::bitset<64> R_without_I)
{
    double bound = 0;
    if (lower_bound)
    {
        /** @todo do this right */
        bound = get_or_compute_tsp_bound(node, R_without_I);
    }
}