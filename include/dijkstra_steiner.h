#pragma once

#include "steinergraph.h"
#include <iostream>
#include <optional>
#include <vector>
#include <functional>
#include <set>
#include <bitset>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <utility>

class DijkstraSteiner
{
public:
    DijkstraSteiner(const SteinerGraph &graph);
    SteinerGraph compute_optimal_steiner_tree(const SteinerGraph::NodeId r0, const bool lower_bound);

    /** @todo make private */
    using TerminalSubset = std::bitset<64>;

private:
    SteinerGraph _graph;

    // for dynamic distance computations
    std::vector<std::vector<int>> _distance_matrix;
    bool _computed_distance_matrix = false;
    void compute_distances_and_check_connected();
    int get_or_compute_distance(const SteinerGraph::NodeId node1, const SteinerGraph::NodeId node2);

    // helper types
    template <typename T, typename U>
    struct PairHash
    {
    public:
        std::size_t operator()(const std::pair<T, U> &x) const;
    };
    template <typename T, typename U, typename V>
    struct TripleHash
    {
    public:
        std::size_t operator()(const std::tuple<T, U, V> &x) const;
    };

    using EdgeTuple = std::tuple<SteinerGraph::NodeId, SteinerGraph::NodeId, int>;
    using LabelKey = std::pair<SteinerGraph::NodeId, TerminalSubset>;
    using WeightedLabelKey = std::pair<double, LabelKey>;
    using LabelKeyToDoubleMap = std::unordered_map<LabelKey, double, PairHash<SteinerGraph::NodeId, TerminalSubset>>;
    using LabelKeyToWeightedLabelKeyVectorMap = std::unordered_map<LabelKey, std::vector<WeightedLabelKey>, PairHash<SteinerGraph::NodeId, TerminalSubset>>;
    using LabelKeySet = std::unordered_set<LabelKey, PairHash<SteinerGraph::NodeId, TerminalSubset>>;

    struct CompareWeightedLabelKey
    {
    public:
        bool operator()(const WeightedLabelKey &a,
                        const WeightedLabelKey &b) const;
    };

    const std::function<bool(const SteinerGraph::NodeId)> is_in_terminal_subset(const TerminalSubset &terminal_subset) const;
    TerminalSubset one_element_terminal_subset(const SteinerGraph::TerminalId terminal_id) const;
    bool is_terminal_subset_of(
        const TerminalSubset &subset,
        const TerminalSubset &superset)
        const;

    // j-terminal bound
    /**
     * stores the j value for which the j-terminal bound has been computed
     * to ensure that the j value stays the same */
    std::optional<int> _computed_j_terminal_bound_j_value;
    /** stores the computed j-terminal bounds */
    LabelKeyToDoubleMap _computed_j_terminal_bounds;
    double get_or_compute_j_terminal_bound(
        const int j,
        const SteinerGraph::NodeId node,
        const TerminalSubset &terminal_subset);

    // one-tree bound

    /**
     * stores the root terminal for which the 1-tree bound has been computed
     * to ensure that the terminal r0 stays the same
     */
    std::optional<SteinerGraph::TerminalId> _computed_one_tree_bound_root_terminal;
    /** stores the computed 1-tree bounds */
    LabelKeyToDoubleMap _computed_one_tree_bounds;
    double get_or_compute_one_tree_bound(
        const SteinerGraph::NodeId node,
        const TerminalSubset &terminal_subset,
        const SteinerGraph::TerminalId r0);

    /** @todo check if all functions are really used */
    // tsp bound
    using HamiltonianPathKey = std::tuple<SteinerGraph::NodeId, SteinerGraph::NodeId, TerminalSubset>;
    using HamiltonianPathKeyToDoubleMap = std::unordered_map<HamiltonianPathKey, double, TripleHash<SteinerGraph::TerminalId, SteinerGraph::TerminalId, TerminalSubset>>;
    HamiltonianPathKeyToDoubleMap _hamiltonian_paths;
    double get_hamiltonian_path(const HamiltonianPathKey &key) const;
    /** stores whether the hamiltonian path lengths have already been computed */
    bool is_hamiltonian_path_computed = false;
    void compute_hamiltonian_paths();
    LabelKeyToDoubleMap _computed_tsp_bounds;
    /** stores the computed tsp bounds */
    double get_or_compute_tsp_bound(
        const SteinerGraph::NodeId node,
        const TerminalSubset &terminal_subset);

    // total bound
    double bound(const bool lower_bound, const SteinerGraph::NodeId node, const TerminalSubset &R_without_I);

    // Dijkstra-Steiner algorithm
    std::vector<EdgeTuple> backtrack(const LabelKeyToWeightedLabelKeyVectorMap &backtrack, const LabelKey &current_label) const;

    /** @todo remove this */
    // testing
    void test_one_tree_bound();
    void test_tsp_bound();
};
