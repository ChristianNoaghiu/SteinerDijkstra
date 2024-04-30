cmake_minimum_required(VERSION 3.14)
project(DijkstraSteiner-Intro)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
add_compile_options(-D_GLIBCXX_DEBUG)

include_directories(include)
add_subdirectory(src)
