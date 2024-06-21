// steinergraph_tsp_bound.cpp (computing the tsp-bound from the Dijkstra-Steiner algorithm)
#include <unordered_map>
#include <tuple>
#include <functional>
#include "steinergraph.h"

/**
 * returns the 1-tree-bound from the Dijkstra-Steiner algorithm
 */
double SteinerGraph::get_or_compute_tsp_bound(
    const NodeId node,
    const SteinerGraph::TerminalSubset &terminal_subset)
{
    // check validity of parameters
    check_valid_node(node);

    // check if the bound has already been computed and return it
    const BoundKey bound_key = std::make_pair(node, terminal_subset);
    if (!is_hamiltonian_path_computed)
    {
        compute_hamiltonian_paths();
    }
    // if no hamiltonian path has been computed yet, there is no need to search the map
    else if (_computed_tsp_bounds.count(bound_key) > 0)
    {
        return _computed_tsp_bounds[bound_key];
    }

    const MetricClosureStruct metric_closure_result = metric_closure();
    const std::vector<std::vector<int>> &distance_matrix = metric_closure_result.distance_matrix;
    check_connected_metric_closure(distance_matrix);

    // compute the minimum of d(v,i) + d(v,j) + hp(i, j, I) as in the definition of the tsp bound
    double result = std::numeric_limits<double>::max();
    // iterate over all elements of the terminal_subset
    for (TerminalId i = 0; i < num_terminals(); i++)
    {
        if (!terminal_subset[i])
        {
            continue;
        }

        for (TerminalId j = 0; j < num_terminals(); j++)
        {
            /** @todo must we check i == j??
             * since a hamiltonian path is not possible with only two nodes (node and i == j),
             * but (1.2) in the paper does not exclude this case
             *
             * and what about the case when node is an element of terminal_subset?
             * then, the edge which belongs to distance_matrix[node][node_j] could be part of the hamiltonian path
             * (e.g. if |I| = 2), so the Hamiltonian cycle would use an edge twice
             */
            if (i == j || !terminal_subset[j])
            {
                continue;
            }
            // convert TerminalId to NodeId for distance_matrix
            NodeId node_i = _terminals_vector.at(i);
            NodeId node_j = _terminals_vector.at(j);

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
    _computed_tsp_bounds[bound_key] = result;
    return result;
}

/**
 * returns the hamiltonian path value (stored in _hamiltonian_paths) for a given key
 * or infinity if no entry has been created yet
 */
double SteinerGraph::get_hamiltonian_path(const SteinerGraph::HamiltonianPathKey &key) const
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
void SteinerGraph::compute_hamiltonian_paths()
{
    // this must happen at the beginning of the method, since otherwise
    // get_hamiltonian_path would return infinity inside the algorithm
    // even though some keys have already been computed
    is_hamiltonian_path_computed = true;

    // compute the distance matrix
    const MetricClosureStruct metric_closure_result = metric_closure();
    const std::vector<std::vector<int>> &distance_matrix = metric_closure_result.distance_matrix;
    check_connected_metric_closure(distance_matrix);

    // initialize start values
    for (TerminalId i = 0; i < num_terminals(); i++)
    {
        _hamiltonian_paths[std::make_tuple(i, i, one_element_terminal_subset(i))] = 0;
    }

    // subsets_of_size.at(n) stores all ullongs of terminal subsets of size n
    std::vector<std::vector<unsigned long long>> subsets_of_size(num_terminals() + 1);

    /** @todo what if num_terminals exceeds 64? Or is exactly 64? */
    for (unsigned long long terminal_subset_ullong = 0; terminal_subset_ullong < (1ULL << num_terminals()); terminal_subset_ullong++)
    {
        subsets_of_size.at(TerminalSubset(terminal_subset_ullong).count()).push_back(terminal_subset_ullong);
    }

    for (int n = 2; n <= num_terminals(); n++)
    {
        // iterate over all subsets of terminal_subset of size n
        /** @todo is this possible faster? */
        for (unsigned long long terminal_subset_ullong : subsets_of_size.at(n))
        {
            // create the corresponding TerminalSubset
            TerminalSubset terminal_subset = terminal_subset_ullong;

            // iterate over all elements i, j of terminal_subset
            for (TerminalId i = 0; i < num_terminals(); i++)
            {
                if (!terminal_subset[i])
                {
                    continue;
                }

                for (TerminalId j = 0; j < num_terminals(); j++)
                {
                    if (!terminal_subset[j])
                    {
                        continue;
                    }

                    // convert TerminalId to NodeId for distance_matrix
                    NodeId node_j = _terminals_vector.at(j);

                    // set this key to infinity
                    HamiltonianPathKey current_key = std::make_tuple(i, j, terminal_subset);

                    // key without j
                    TerminalSubset terminal_subset_without_j = terminal_subset;
                    terminal_subset_without_j[j] = 0;

                    for (TerminalId x = 0; x < num_terminals(); x++)
                    {
                        if (!terminal_subset[x] || x == j)
                        {
                            continue;
                        }

                        NodeId node_x = _terminals_vector.at(x);

                        // compare and set the new value of the key
                        HamiltonianPathKey key_without_j = std::make_tuple(i, x, terminal_subset_without_j);
                        double hamiltonian_path_without_j = get_hamiltonian_path(key_without_j);
                        double distance_x_j = distance_matrix[node_x][node_j];
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

void SteinerGraph::test_tsp_bound()
{
    compute_hamiltonian_paths();
    std::cout << get_hamiltonian_path(std::make_tuple(0, 0, 0b001)) << "\n";
    std::cout << get_hamiltonian_path(std::make_tuple(0, 1, 0b011)) << "\n";
    std::cout << get_or_compute_tsp_bound(0, 0b011) << "\n";
}