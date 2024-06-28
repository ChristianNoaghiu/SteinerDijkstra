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
        auto start = std::chrono::high_resolution_clock::now();
        SteinerGraph dijkstra_steiner_result = algorithm.compute_optimal_steiner_tree(0, true);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";

        dijkstra_steiner_result.print_weight();
    }
}
