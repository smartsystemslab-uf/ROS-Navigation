//
// Created by afelder on 10/1/18.
//

#ifndef GRAPHLIBRARY_EDGE_H
#define GRAPHLIBRARY_EDGE_H

#include "Node.h"


template <class T>
class Edge {
public:

    Edge(const Node<T> &a, const Node<T> &b){
        nodeA = a;
        nodeB = b;
    }


    friend bool operator <(Edge a, Edge b){return a.nodeA.getNodeID() < b.nodeA.getNodeID() || a.nodeB.getNodeID() < b.nodeB.getNodeID();}

    void printEdge(){
           cout << "First Node: " << nodeA.getNodeID() << " Second Node: " << nodeB.getNodeID() << endl;
    }

    const Node<T> &getNodeA() const {
        return nodeA;
    }

    void setNodeA(const Node<T> &nodeA) {
        Edge::nodeA = nodeA;
    }

    const Node<T> &getNodeB() const {
        return nodeB;
    }

    void setNodeB(const Node<T> &nodeB) {
        Edge::nodeB = nodeB;
    }

    float getCost() const {
        return cost;
    }

    void setCost(float cost) {
        Edge::cost = cost;
    }

private:
    Node<T> nodeA;
    Node<T> nodeB;

    float cost;
};


#endif //GRAPHLIBRARY_EDGE_H
