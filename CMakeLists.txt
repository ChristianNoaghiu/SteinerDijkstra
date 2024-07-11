cmake_minimum_required(VERSION 3.16)
project(DijkstraSteiner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

include_directories(include)
include_directories(external)

file(GLOB SRC_FILES
${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(Steinertree_both.exe main/testgraph.cpp ${SRC_FILES})