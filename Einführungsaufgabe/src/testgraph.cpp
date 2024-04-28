// testgraph.cpp (Read Digraph from File and Print)

#include "steinergraph.h"

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        SteinerGraph g(argv[1]);

        std::vector<std::vector<int>> distance_matrix(g.num_nodes(), std::vector<int>(g.num_nodes()));
        std::vector<std::vector<SteinerGraph::NodeId>> predecessor_matrix(g.num_nodes(), std::vector<SteinerGraph::NodeId>(g.num_nodes()));

        g.metric_closure(distance_matrix, predecessor_matrix);

        g.print();
    }
}
