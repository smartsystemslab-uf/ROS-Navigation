#include <iostream>
#include "GraphLibrary/DataStructures/Graph.h"


// TODO Generate Random Graph
// TODO



int main() {
    std::cout << "Hello, World!" << std::endl;
    Rand r(73);
    vector<int> dims = {5,5, 300, 300};
    vector<int> minMaxChildren = {3,6};
    vector<float> neighborProbs = {};

    Graph<int> *g = Graph<int>::genRandomMultiLayerGraph(r, dims, 2, minMaxChildren);

    cout << g->getNumNodes() << " Nodes" << endl;
    cout << g->getNumEdges() <<  " Edges" << endl;

    for(Node<int> node : g->getNodes()){
        cout << node.getChildGraph()->getNumNodes() << " ";
        cout << node.getChildGraph()->getNumEdges() << endl;
    }




    delete g;
    return 0;
}