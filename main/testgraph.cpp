// testgraph.cpp (Read Digraph from File and Print)

#include "steinergraph.h"
#include "dijkstra_steiner.h"
#include <iostream>

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        SteinerGraph g(argv[1]);
        DijkstraSteiner algorithm(g);
        SteinerGraph dijkstra_steiner_result = algorithm.compute_optimal_steiner_tree(g.find_terminal_node().value(), true);

        dijkstra_steiner_result.print_weight();
    }
}
