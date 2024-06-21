#include "steinergraph.h"

int SteinerGraph::terminal_subset_size(const SteinerGraph::TerminalSubset &terminal_subset)
{
    int size = 0;
    /** @todo do this by shifting */
    for (unsigned int i = 0; i < terminal_subset.size(); i++)
    {
        if (terminal_subset[i])
        {
            size++;
        }
    }
    return size;
}