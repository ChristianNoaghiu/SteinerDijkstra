#include "dijkstra_steiner.h"

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
template <typename T, typename U>
std::size_t DijkstraSteiner::PairHash<T, U>::operator()(const std::pair<T, U> &x) const
{
    return std::hash<T>()(x.first) ^ std::hash<U>()(x.second.to_ullong());
}

/**
 * hash function for a map of 3-tuples
 */
template <typename T, typename U, typename V>
::std::size_t DijkstraSteiner::TripleHash<T, U, V>::operator()(const std::tuple<T, U, V> &x) const
{
    return std::hash<T>()(std::get<0>(x)) ^ std::hash<U>()(std::get<1>(x)) ^ std::hash<V>()(std::get<2>(x).to_ullong());
}

// Explicit instantiation of the used template structs
template struct DijkstraSteiner::PairHash<SteinerGraph::NodeId, DijkstraSteiner::TerminalSubset>;
template struct DijkstraSteiner::TripleHash<SteinerGraph::TerminalId, SteinerGraph::TerminalId, DijkstraSteiner::TerminalSubset>;

/**
 * comparison function for a priority queue of weighted label keys
 */
bool DijkstraSteiner::CompareWeightedLabelKey::operator()(const DijkstraSteiner::WeightedLabelKey &a,
                                                          const DijkstraSteiner::WeightedLabelKey &b) const
{
    return a.first > b.first; // Vergleicht nur den double-Wert, also die Distanz
}