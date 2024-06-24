// testgraph.cpp (Read Digraph from File and Print)

#include "steinergraph.h"
#include <iostream>

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        std::cout << "Hi" << std::endl;
        SteinerGraph g(argv[1]);
        SteinerGraph dijkstra_steiner_result = g.dijkstra_steiner(0, false);

        dijkstra_steiner_result.print();

        /*std::cout << dijkstra_steiner_result.size() << std::endl;

        for (auto &pair : dijkstra_steiner_result)
        {
            std::cout << pair.first + 1 << " <-> " << pair.second + 1 << ", weight: " << g.get_node(pair.first).adjacent_nodes().find(pair.second) << std::endl;
        }
        std::cout << "check" << std::endl;*/

        /** @todo remove this */
        // g.test_one_tree_bound();

        /** @todo execute complete algorithm */
        // SteinerGraph h = g.steiner_tree_mst_approximation();

        // std::cout << "-- Printing MST approximation for Steiner tree --\n\n";
        // h.print();
    }
}
