// steinergraph_lambdas.cpp (helper lambdas in SteinerGraph)
#include "steinergraph.h"

/**
 * returns a lambda which compares the distances of two NodeDistancePairs
 * according to the second component
 */
std::function<bool(
    const SteinerGraph::NodeDistancePair,
    const SteinerGraph::NodeDistancePair)>
SteinerGraph::node_distance_pair_compare()
{
    return [](const NodeDistancePair &node_distance1,
              const NodeDistancePair &node_distance2)
    {
        const int distance1 = node_distance1.second;
        const int distance2 = node_distance2.second;

        return (distance1 > distance2);
    };
}

/**
 * lambda returning true for all nodes
 * (whole graph as subgraph)
 */
const std::function<bool(const SteinerGraph::NodeId)> SteinerGraph::is_in_graph() const
{
    return [](__attribute__((unused)) const SteinerGraph::NodeId node)
    {
        /**
         * @todo Wunused complains about node not being used,
         * therefore this redundant comparison
         */
        return true;
    };
}

/**
 * lambda returning whether a node is contained in node_set
 * (for induced subgraph)
 */
const std::function<bool(const SteinerGraph::NodeId)> SteinerGraph::is_in_set(const std::set<SteinerGraph::NodeId> &node_set) const
{
    return [node_set = node_set](const SteinerGraph::NodeId node)
    {
        return node_set.count(node) != 0;
    };
}

const std::function<bool(const SteinerGraph::NodeId)> SteinerGraph::is_in_terminal_subset(const SteinerGraph::TerminalSubset &terminal_subset) const
{
    return [&terminal_subset = terminal_subset, &terminals = _terminals_vector](const SteinerGraph::NodeId node)
    {
        auto terminal_iterator = std::find(terminals.begin(), terminals.end(), node);

        if (terminal_iterator == terminals.end())
        {
            return false;
        }

        const TerminalId terminal_id = std::distance(terminals.begin(), terminal_iterator);

        if (static_cast<size_t>(terminal_id) >= terminal_subset.size())
        {
            throw std::runtime_error("TerminalId exceeds the size of terminal_subset");
        }

        return terminal_subset[terminal_id] != 0;
    };
}