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
    return [](const SteinerGraph::NodeId node)
    {
        /**
         * @todo Wunused complains about node not being used,
         * therefore this redundant comparison
         */
        if (node + 1 == node + 1)
            return true;
        return true;
    };
}

/**
 * lambda returning whether a node is contained in node_set
 * (for induced subgraph)
 */
const std::function<bool(const SteinerGraph::NodeId)> SteinerGraph::is_in_set(const std::unordered_set<SteinerGraph::NodeId> &node_set) const
{
    return [node_set = node_set](const SteinerGraph::NodeId node)
    {
        return node_set.count(node) != 0;
    };
}