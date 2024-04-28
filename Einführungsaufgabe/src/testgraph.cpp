// testgraph.cpp (Read Digraph from File and Print)

#include "steinergraph.h"

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        SteinerGraph g(argv[1]);

        SteinerGraph h = g.steiner_tree_mst_approximation();

        h.print();
    }
}
