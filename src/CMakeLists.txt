include_directories(${DijkstraSteiner-Intro_SOURCE_DIR}/include)
add_executable(Testgraph testgraph.cpp steinergraph.cpp steinergraph_mst.cpp steinergraph_one_tree_bound.cpp)