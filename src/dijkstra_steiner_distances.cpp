#include "dijkstra_steiner.h"

/**
 * computes all distances between nodes and checks whether the graph is connected
 * stores the result in _distance_matrix
 */
void DijkstraSteiner::compute_distances_and_check_connected()
{
    // check if the distances have already been computed
    if (_computed_distance_matrix)
    {
        return;
    }

    const SteinerGraph::MetricClosureStruct metric_closure_result = _graph.metric_closure();
    _distance_matrix = metric_closure_result.distance_matrix;
    _graph.check_connected_metric_closure(_distance_matrix);
    _computed_distance_matrix = true;
}

/**
 * returns the distance between two nodes dynamically
 */
int DijkstraSteiner::get_or_compute_distance(
    const SteinerGraph::NodeId node1, const SteinerGraph::NodeId node2)
{
    compute_distances_and_check_connected();
    return _distance_matrix.at(node1).at(node2);
}