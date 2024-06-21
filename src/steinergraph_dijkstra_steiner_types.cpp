#include "steinergraph.h"

/**
 * returns the amount of terminals in a TerminalSubset
 */
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

/**
 * returns a TerminalSubset containing exactly the given terminal
 */
SteinerGraph::TerminalSubset SteinerGraph::one_element_terminal_subset(const SteinerGraph::TerminalId terminal_id) const
{
    check_valid_terminal(terminal_id);
    TerminalSubset terminal_subset;
    terminal_subset[terminal_id] = 1;
    return terminal_subset;
}