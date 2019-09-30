#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

/* Note: need to pass robotSize, bufferSize, gridSize */

class Node {
  public:
    int x, y; // coordinates of the Node in the image
    double cost; // absolute cost (based on the color), FIXME
    double cost2; // relative cost (distance to goal), FIXME
    Node *parent; // parent of Node in path/algorithm

  Node() {
    // FIXME
  }

  Node(/* FIXME */) {

  }

  ~Node() {
  }

  /* determine if the Node is close enough to the goal (if not possible to exactly end on goal) */
  bool isCloseEnough(/* goal, robotSize */) {

  }

  /* h(n), heuristic that estimates the cost of the cheapest path from n to the goal */
  double hCost(/* goal */) {

  }

  /* g(n), cost of the path from the start node to n */
  double gCost(/* node */) {

  }

  /* finds valid neighbors */
  void findNeighbors(/* vector of Nodes passed by reference */) {

  }

}
