// steinergraph_tsp_bound.cpp (computing the tsp-bound from the Dijkstra-Steiner algorithm)
#include "dijkstra_steiner.h"
#include "steinergraph.h"

/**
 * returns the 1-tree-bound from the Dijkstra-Steiner algorithm
 */
double DijkstraSteiner::get_or_compute_tsp_bound(
    const SteinerGraph::NodeId node,
    const DijkstraSteiner::TerminalSubset &terminal_subset)
{
    // check validity of parameters
    _graph.check_valid_node(node);

    // check if the bound has already been computed and return it
    const LabelKey label_key = std::make_pair(node, terminal_subset);
    if (!is_hamiltonian_path_computed)
    {
        compute_hamiltonian_paths();
    }
    // if no hamiltonian path has been computed yet, there is no need to search the map
    else if (_computed_tsp_bounds.count(label_key) > 0)
    {
        return _computed_tsp_bounds[label_key];
    }

    const SteinerGraph::MetricClosureStruct metric_closure_result = _graph.metric_closure();
    const std::vector<std::vector<int>> &distance_matrix = metric_closure_result.distance_matrix;
    _graph.check_connected_metric_closure(distance_matrix);

    // compute the minimum of d(v,i) + d(v,j) + hp(i, j, I) as in the definition of the tsp bound
    double result = std::numeric_limits<double>::max();
    // iterate over all elements of the terminal_subset
    for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
    {
        if (!terminal_subset[i])
        {
            continue;
        }

        for (SteinerGraph::TerminalId j = 0; j < _graph.num_terminals(); j++)
        {
            // we exclude the case where i == j, since a hamiltonian path is not possible with only two nodes (node and i == j)
            // and we exclude that j is interminalsubset, since the hamiltonian cycle would use an edge twice
            if (i == j || !terminal_subset[j])
            {
                continue;
            }
            // convert TerminalId to NodeId for distance_matrix
            SteinerGraph::NodeId node_i = _graph.get_terminals().at(i);
            SteinerGraph::NodeId node_j = _graph.get_terminals().at(j);

            // compute the length of the Hamiltonian path from i to j
            HamiltonianPathKey hamilton_path_key = std::make_tuple(i, j, terminal_subset);
            double hamiltonian_path = get_hamiltonian_path(hamilton_path_key);

            // compute the distance for closing the Hamiltonian cycle
            double distance_node_i_j = distance_matrix[node][node_i] + distance_matrix[node][node_j];

            // prevent overflow
            if (hamiltonian_path >= std::numeric_limits<double>::max() - distance_node_i_j)
            {
                continue;
            }

            // update result
            double new_result = 0.5 * (distance_node_i_j + hamiltonian_path);
            if (new_result < result)
            {
                result = new_result;
            }
        }
    }

    // store the result dynamically
    _computed_tsp_bounds[label_key] = result;
    return result;
}

/**
 * returns the hamiltonian path value (stored in _hamiltonian_paths) for a given key
 * or infinity if no entry has been created yet
 */
double DijkstraSteiner::get_hamiltonian_path(const DijkstraSteiner::HamiltonianPathKey &key) const
{
    if (!is_hamiltonian_path_computed || _hamiltonian_paths.count(key) == 0)
    {
        return std::numeric_limits<double>::max();
    }
    return _hamiltonian_paths.find(key)->second;
}

/**
 * computes hp as in the provided algorithm for the tsp bound
 * and stores the result in _hamiltonian_paths
 */
void DijkstraSteiner::compute_hamiltonian_paths()
{
    // this must happen at the beginning of the method, since otherwise
    // get_hamiltonian_path would return infinity inside the algorithm
    // even though some keys have already been computed
    is_hamiltonian_path_computed = true;

    // initialize start values
    for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
    {
        _hamiltonian_paths[std::make_tuple(i, i, one_element_terminal_subset(i))] = 0;
    }

    for (int n = 2; n <= _graph.num_terminals(); n++)
    {
        // iterate over all subsets of terminal_subset of size n
        for (const TerminalSubset &terminal_subset : _terminal_subsets_of_size.at(n))
        {
            // iterate over all elements i, j of terminal_subset
            for (SteinerGraph::TerminalId i = 0; i < _graph.num_terminals(); i++)
            {
                if (!terminal_subset[i])
                {
                    continue;
                }

                for (SteinerGraph::TerminalId j = 0; j < _graph.num_terminals(); j++)
                {
                    if (!terminal_subset[j])
                    {
                        continue;
                    }

                    // convert TerminalId to NodeId for distance_matrix
                    SteinerGraph::NodeId node_j = _graph.get_terminals().at(j);

                    // set this key to infinity
                    HamiltonianPathKey current_key = std::make_tuple(i, j, terminal_subset);

                    // key without j
                    TerminalSubset terminal_subset_without_j = terminal_subset;
                    terminal_subset_without_j[j] = 0;

                    for (SteinerGraph::TerminalId x = 0; x < _graph.num_terminals(); x++)
                    {
                        if (!terminal_subset[x] || x == j)
                        {
                            continue;
                        }

                        SteinerGraph::NodeId node_x = _graph.get_terminals().at(x);

                        // compare and set the new value of the key
                        HamiltonianPathKey key_without_j = std::make_tuple(i, x, terminal_subset_without_j);
                        double hamiltonian_path_without_j = get_hamiltonian_path(key_without_j);
                        double distance_x_j = get_or_compute_distance(node_x, node_j);
                        // prevent overflow
                        if (hamiltonian_path_without_j >= std::numeric_limits<double>::max() - distance_x_j)
                        {
                            continue;
                        }

                        // otherwise, computing new_value is safe
                        double new_value = hamiltonian_path_without_j + distance_x_j;
                        if (get_hamiltonian_path(current_key) > new_value)
                        {
                            _hamiltonian_paths[current_key] = new_value;
                        }
                    }
                }
            }
        }
    }
}
