#!/bin/bash

rm -rf CMakeFiles
rm cmake_install.cmake
rm CMakeCache.txt
rm PathFindingTest
cmake .
make
clear
clear
cmake .
make
