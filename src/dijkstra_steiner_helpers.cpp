#include "dijkstra_steiner.h"
#include <limits>

/**
 * returns the bitset-input -1 as a bitset again, for arbitrary bitset length
 * use: go through all subsets of a given bitset by subtracting 1 from the bitset until it is 0
 */
DijkstraSteiner::TerminalSubset DijkstraSteiner::minus_one(const DijkstraSteiner::TerminalSubset &input)
{
    const TerminalSubset mask = -1ULL;
    TerminalSubset result = 0;
    int i = 0;
    const int width = std::numeric_limits<unsigned long long>::digits;
    // iterate through the bitset in 64-bit steps (words), so we can use to_ullong() to get the 64-bit integer and subtract 1 fast
    // for that, we need to invert all bits up to the first 1 (it included) of the input
    while (i <= bitset_length)
    {
        if (!((result >> i) & mask).any())
        {
            // input has to be all zeros in this word for this case to happen
            result ^= mask << i;
            i += width;
            continue;
        }
        // if we are here, we have found a 1 in the current (<=64-bit) word and will use to_ullong for fast subtraction
        const TerminalSubset temp = (((input >> i) & mask).to_ullong() - 1) << i;
        result ^= temp;
        result ^= (input >> (i + width)) << (i + width); // copy the rest of the input
        break;
    }
    return result;
}

/**
 * returns a TerminalSubset containing exactly the given terminal
 */
DijkstraSteiner::TerminalSubset DijkstraSteiner::one_element_terminal_subset(const SteinerGraph::TerminalId terminal_id) const
{
    _graph.check_valid_terminal(terminal_id);
    return (TerminalSubset(1) << terminal_id);
}

/**
 * lambda returning whether a node is contained in a terminal subset
 * (for which it being a terminal is necessary)
 */
const std::function<bool(const SteinerGraph::NodeId)> DijkstraSteiner::is_in_terminal_subset(const DijkstraSteiner::TerminalSubset &terminal_subset) const
{
    return [&terminal_subset = terminal_subset, &terminals = _graph.get_terminals()](const SteinerGraph::NodeId node)
    {
        auto terminal_iterator = std::find(terminals.begin(), terminals.end(), node);

        if (terminal_iterator == terminals.end())
        {
            return false;
        }

        const SteinerGraph::TerminalId terminal_id = std::distance(terminals.begin(), terminal_iterator);

        if (static_cast<size_t>(terminal_id) >= terminal_subset.size())
        {
            throw std::runtime_error("TerminalId exceeds the size of terminal_subset");
        }

        return terminal_subset[terminal_id] != 0;
    };
}

/**
 * returns whether subset is a subset of superset
 */
bool DijkstraSteiner::is_terminal_subset_of(const SteinerGraph::TerminalSubset &subset, const SteinerGraph::TerminalSubset &superset) const
{
    for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
    {
        if (subset[i] && !superset[i])
        {
            return false;
        }
    }
    return true;
}

/**
 * hash function for a map of pairs
 */
std::size_t DijkstraSteiner::PairHash::operator()(const std::pair<SteinerGraph::NodeId, DijkstraSteiner::TerminalSubset> &x) const
{
    return std::hash<SteinerGraph::NodeId>()(x.first) ^ std::hash<DijkstraSteiner::TerminalSubset>()(x.second);
}

/**
 * hash function for a map of 3-tuples
 */
::std::size_t DijkstraSteiner::TripleHash::operator()(const std::tuple<SteinerGraph::TerminalId, SteinerGraph::TerminalId, DijkstraSteiner::TerminalSubset> &x) const
{
    return std::hash<SteinerGraph::TerminalId>()(std::get<0>(x)) ^ std::hash<SteinerGraph::TerminalId>()(std::get<1>(x)) ^ std::hash<DijkstraSteiner::TerminalSubset>()(std::get<2>(x));
}

/**
 * comparison function for a priority queue of weighted label keys
 */
bool DijkstraSteiner::CompareWeightedLabelKey::operator()(const DijkstraSteiner::WeightedLabelKey &a,
                                                          const DijkstraSteiner::WeightedLabelKey &b) const
{
    return a.first > b.first; // Vergleicht nur den double-Wert, also die Distanz
}