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

std::function<bool(
    const SteinerGraph::EdgeTuple,
    const SteinerGraph::EdgeTuple)>
SteinerGraph::edge_tuple_compare()
{
    return [](const SteinerGraph::EdgeTuple &edge_tuple1,
              const SteinerGraph::EdgeTuple &edge_tuple2)
    {
        const int distance1 = std::get<2>(edge_tuple1);
        const int distance2 = std::get<2>(edge_tuple2);

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
        return true;
    };
}

/**
 * lambda returning true for all terminals
 * (whole graph as subgraph)
 */
const std::function<bool(const SteinerGraph::NodeId)> SteinerGraph::is_terminal() const
{
    return [&terminals = _terminals](__attribute__((unused)) const SteinerGraph::NodeId node)
    {
        return std::find(terminals.begin(), terminals.end(), node) != terminals.end();
    };
}