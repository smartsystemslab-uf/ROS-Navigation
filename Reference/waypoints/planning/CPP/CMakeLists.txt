cmake_minimum_required(VERSION 3.12)
project(CPP)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE GRAPH_LIBRARY_H "GraphLibrary/*.h")
file(GLOB_RECURSE GRAPH_LIBRARY_CPP "GraphLibrary/*.cpp")

add_executable(CPP main.cpp ${GRAPH_LIBRARY_H} ${GRAPH_LIBRARY_CPP})