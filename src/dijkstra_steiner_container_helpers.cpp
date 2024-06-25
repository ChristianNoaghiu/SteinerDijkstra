#include "dijkstra_steiner.h"

/**
 * returns a TerminalSubset containing exactly the given terminal
 */
DijkstraSteiner::TerminalSubset DijkstraSteiner::one_element_terminal_subset(const SteinerGraph::TerminalId terminal_id) const
{
    _graph.check_valid_terminal(terminal_id);
    TerminalSubset terminal_subset;
    terminal_subset[terminal_id] = 1;
    return terminal_subset;
}

/**
 * lambda returning whether a node is contained in a terminal subset
 * (for which being a terminal is necessary)
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
 * hash function for a map of pairs
 */
struct DijkstraSteiner::PairHash
{
public:
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U> &x) const
    {
        return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
    }
};

/**
 * hash function for a map of 3-tuples
 */
struct DijkstraSteiner::TripleHash
{
public:
    template <typename T, typename U, typename V>
    std::size_t operator()(const std::tuple<T, U, V> &x) const
    {
        return std::hash<T>()(std::get<0>(x)) ^ std::hash<U>()(std::get<1>(x)) ^ std::hash<V>()(std::get<2>(x));
    }
};

/**
 * comparison function for a priority queue of weighted label keys
 */
struct DijkstraSteiner::CompareWeightedLabelKey
{
    bool operator()(const WeightedLabelKey &a,
                    const WeightedLabelKey &b) const
    {
        return a.first > b.first; // Vergleicht nur den double-Wert, also die Distanz
    }
};