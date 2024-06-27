#include "dijkstra_steiner.h"

/**
 * returns the bitset-input -1 as a bitset again, for arbitrary bitset length
 * use: go through all subsets of a given bitset by subtracting 1 from the bitset until it is 0
 */
DijkstraSteiner::TerminalSubset DijkstraSteiner::minus_one(const DijkstraSteiner::TerminalSubset &input)
{
    int sixtyfour = 64;
    // if bitset_length is <64 the TerminalSubset below should be all 1's, else only the first 64 bits
    if (bitset_length < 64)
    {
        sixtyfour = bitset_length;
    }
    const TerminalSubset twotothesixth_ones = TerminalSubset().set() >> (bitset_length - sixtyfour);
    const int bitset_length_helper = bitset_length - 64;
    TerminalSubset result = 0;
    int i = 0;
    // iterate through the bitset in 64-bit steps, so we can use to_ullong() to get the 64-bit integer and subtract 1 fast
    // for that, we need to invert all bits up to the first 1 (it included) of the input
    while (i <= bitset_length_helper)
    {
        if (!((result >> i) & twotothesixth_ones).any())
        {
            result ^= twotothesixth_ones << i;
            i += 64;
            continue;
        }
        // if we are here, we have found a first 1 in the current 64-bit block and will use to_ullong for fast subtraction
        const TerminalSubset temp = (((input >> i) & twotothesixth_ones).to_ullong() - 1) << i;
        result ^= temp;
        result ^= (input >> (i + 64)) << (i + 64);
        return result;
    }
    // if we are here, we are in the last block of input containing < 64 bits and there has been no 1 in input so far
    result ^= ((input >> i).to_ullong() - 1) << i;
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