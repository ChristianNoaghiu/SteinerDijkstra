// testgraph.cpp (Read Digraph from File and Print)

#include "steinergraph.h"
#include "dijkstra_steiner.h"
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        SteinerGraph g(argv[1]);
        DijkstraSteiner algorithm(g);

        auto start1 = std::chrono::high_resolution_clock::now();
        SteinerGraph dijkstra_steiner_result = algorithm.compute_optimal_steiner_tree(g.find_terminal_node().value(), true);
        auto end1 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> elapsed1 = end1 - start1;
        std::cout << "Overall Elapsed time: " << elapsed1.count() << " s\n";

        dijkstra_steiner_result.print_weight();
    }
}
