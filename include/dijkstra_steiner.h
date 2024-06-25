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

class DijkstraSteiner
{
public:
    SteinerGraph _graph;
    DijkstraSteiner(const SteinerGraph &graph);

    // for dynamic distance computations
    std::vector<std::vector<int>> _distance_matrix;
    bool _computed_distance_matrix = false;
    void compute_distances_and_check_connected();
    int get_or_compute_distance(const SteinerGraph::NodeId node1, const SteinerGraph::NodeId node2);

    // helper types
    struct PairHash;
    struct TripleHash;
    struct CompareWeightedLabelKey;

    using TerminalSubset = std::bitset<64>;
    const std::function<bool(const SteinerGraph::NodeId)> is_in_terminal_subset(const TerminalSubset &terminal_subset) const;
    TerminalSubset one_element_terminal_subset(const SteinerGraph::TerminalId terminal_id) const;

    using EdgeTuple = std::tuple<SteinerGraph::NodeId, SteinerGraph::NodeId, int>;
    using LabelKey = std::pair<SteinerGraph::NodeId, TerminalSubset>;
    using WeightedLabelKey = std::pair<double, LabelKey>;
    using LabelKeyToDoubleMap = std::unordered_map<LabelKey, double, PairHash>;
    using LabelKeyToWeightedLabelKeyVectorMap = std::unordered_map<LabelKey, std::vector<WeightedLabelKey>, PairHash>;
    using LabelKeySet = std::unordered_set<LabelKey, PairHash>;

    // one-tree bound
    std::optional<SteinerGraph::TerminalId> _computed_one_tree_bound_root_terminal;
    LabelKeyToDoubleMap _computed_one_tree_bounds;
    double get_or_compute_one_tree_bound(
        const SteinerGraph::NodeId node,
        const TerminalSubset &terminal_subset,
        const SteinerGraph::TerminalId r0);

    /** @todo check if all functions are really used */
    // tsp bound
    using HamiltonianPathKey = std::tuple<SteinerGraph::NodeId, SteinerGraph::NodeId, TerminalSubset>;
    using HamiltonianPathKeyToDoubleMap = std::unordered_map<HamiltonianPathKey, double, TripleHash>;
    HamiltonianPathKeyToDoubleMap _hamiltonian_paths;
    double get_hamiltonian_path(const HamiltonianPathKey &key) const;
    bool is_hamiltonian_path_computed = false;
    void compute_hamiltonian_paths();
    LabelKeyToDoubleMap _computed_tsp_bounds;
    double get_or_compute_tsp_bound(
        const SteinerGraph::NodeId node,
        const TerminalSubset &terminal_subset);

    // total bound
    double bound(const bool lower_bound, const SteinerGraph::NodeId node, const TerminalSubset &R_without_I);

    // Dijkstra-Steiner algorithm
    SteinerGraph compute_optimal_steiner_tree(const SteinerGraph::NodeId r0, const bool lower_bound);
    std::vector<EdgeTuple> backtrack(const LabelKeyToWeightedLabelKeyVectorMap &backtrack, const LabelKey &current_label) const;

    /** @todo remove this */
    // testing
    void test_one_tree_bound();
    void test_tsp_bound();
};
