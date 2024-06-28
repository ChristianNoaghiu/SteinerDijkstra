#include "dijkstra_steiner.h"
#include <limits>

/**
 * returns the bitset-input -1 as a bitset again, for arbitrary bitset length
 * use: go through all subsets of a given bitset by subtracting 1 from the bitset until it is 0
 */

DijkstraSteiner::TerminalSubset DijkstraSteiner::minus_one(const DijkstraSteiner::TerminalSubset &input)
{
    if (input.none())
    {
        throw std::invalid_argument("input is 0, so we cannot subtract");
    }
    const TerminalSubset mask = -1ULL;
    TerminalSubset result = 0;
    int i = 0;
    const int width = std::numeric_limits<unsigned long long>::digits;
    // iterate in 64-bit steps (words) and use to_ullong() for 64-bit integer and fast subtraction - means bits need to be set to 1 until the first word with 1 is found
    while (i < bitset_length)
    {
        if ((input & (mask << i)).none())
        {
            result ^= mask << i;
            i += width;
            continue;
        }
        // found 1, use to_ullong() for fast -1
        const TerminalSubset temp = (((input >> i) & mask).to_ullong() - 1);
        result ^= temp << i; // I hope this is faster then doing it manually with __builtin_ctzl
        // insert rest of input
        const TerminalSubset temp2 = (input & (mask << (i + width)));
        result ^= temp2;
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
    return [&terminal_subset = terminal_subset, &graph = _graph](const SteinerGraph::NodeId node)
    {
        std::optional<SteinerGraph::TerminalId> terminal_id = graph.find_terminal_id(node);
        if (!terminal_id.has_value())
        {
            return false;
        }

        if (static_cast<size_t>(terminal_id.value()) >= terminal_subset.size())
        {
            throw std::runtime_error("TerminalId exceeds the size of terminal_subset");
        }

        return terminal_subset[terminal_id.value()] != 0;
    };
}

/**
 * returns whether subset is a subset of superset
 */
bool DijkstraSteiner::is_terminal_subset_of(const DijkstraSteiner::TerminalSubset &subset, const DijkstraSteiner::TerminalSubset &superset) const
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