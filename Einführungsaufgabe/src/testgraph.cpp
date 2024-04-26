// testgraph.cpp (Read Digraph from File and Print)

#include "steinergraph.h"

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        SteinerGraph g(argv[1]);
        g.dijkstra(0);
        g.print();
    }
}
