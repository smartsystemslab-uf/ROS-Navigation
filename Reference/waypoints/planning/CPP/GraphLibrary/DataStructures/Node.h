//
// Created by afelder on 10/1/18.
//

#ifndef GRAPHLIBRARY_NODE_H
#define GRAPHLIBRARY_NODE_H


#include <iostream>
#include <cstdlib>
#include <set>
#include <vector>

using std::cout;
using std::set;
using std::endl;
using std::vector;
using std::string;


template <class T>
class Graph;


template <class T>
class Node {
public:
    Node(T val){ value = val;}

    Node(int x_loc, int y_loc, int l, int id, T val){
        x = x_loc;
        y = y_loc;
        level = l;
        nodeID = id;
        value = val;
        containsGraph = false;
    }
    Node(int x_loc, int y_loc, int l, int id){
        x = x_loc;
        y = y_loc;
        level = l;
        nodeID = id;
        containsGraph = false;
    }
    Node(){}

    ~Node(){
//        delete childGraph;
    }

    friend bool operator <(Node a, Node b){return (a.nodeID < b.nodeID) || (a.value < b.value);}
    friend bool operator !=(Node a, Node b){return a.nodeID != b.nodeID;}
    friend bool operator ==(Node a, Node b){return a.nodeID == b.nodeID;}

    T getValue() const {
        return value;
    }

    void setValue(T value) {
        Node::value = value;
    }

    bool addNeighbor(Node<T> n){
        return neighbors.insert(n).second;
    }

    int getX() const {
        return x;
    }

    void setX(int x) {
        Node::x = x;
    }

    int getY() const {
        return y;
    }

    void setY(int y) {
        Node::y = y;
    }

    int getLevel() const {
        return level;
    }

    void setLevel(int level) {
        Node::level = level;
    }

    const set<Node<T>> &getNeighbors() const {
        return neighbors;
    }

    void setNeighbors(const set<Node<T>> &neighbors) {
        Node::neighbors = neighbors;
    }

    int getEdgeCount() const {
        return edgeCount;
    }

    void setEdgeCount(int edgeCount) {
        Node::edgeCount = edgeCount;
    }

    const int &getNodeID() const {
        return nodeID;
    }

    void setNodeID(const int &nodeID) {
        Node::nodeID = nodeID;
    }

    Graph<T> *getChildGraph() const {
        return childGraph;
    }

    void setChildGraph(Graph<T> *childGraph) {
//        delete Node::childGraph;
        Node::childGraph = childGraph;
    }

    bool hasGraph() const {
        return containsGraph;
    }

    void setContainsGraph(bool containsGraph) {
        Node::containsGraph = containsGraph;
    }

    virtual void print(){
        cout << "X: " << x << endl
             << "Y: " << y << endl
             << "Level: " << level << endl
             << "Value: " << value << endl
             << "NodeID: " << nodeID << endl << endl;
    }

    virtual void name(){};

private:
    T value;
    int x;
    int y;
    int level;
    bool containsGraph;

    set<Node<T>> neighbors;
    int edgeCount;

    Graph<T>* childGraph;
    int nodeID;
};

//template <class T>
//class CeilingNode : public Node<T>{
//public:
//    CeilingNode(int x_loc, int y_loc, int l, int id) : Node<T>(x_loc, y_loc, l, id){}
//
//    void name(){cout << "Ceiling Node" << endl;}
//};
//
//
//template <class T>
//class GroundNode : public Node<T>{
//public:
//    GroundNode(int x_loc, int y_loc, int l, int id) : Node<T>(x_loc, y_loc, l, id){}
//
//    void name(){cout << "Ground Node" << endl;}
//};

#endif //GRAPHLIBRARY_NODE_H
