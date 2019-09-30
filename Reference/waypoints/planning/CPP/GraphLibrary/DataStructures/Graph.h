//
// Created by afelder on 10/1/18.
//



#ifndef GRAPHLIBRARY_GRAPH_H
#define GRAPHLIBRARY_GRAPH_H

#include "Node.h"
#include "Edge.h"
#include "../Utility/rand.h"
#include "../Utility/error.h"

#include <set>
#include <vector>
#include <string>
#include <cmath>

using std::cout;
using std::set;
using std::cerr;
using std::vector;
using std::endl;
using std::pair;
using std::to_string;
using std::abs;

//Since this is a template class, all the methods have to be implemented in the .h file.
//So I've put the methods portion of this class at the bottom

template <class T>
class Graph {
public:
    enum graphType{UNDIR_GRAPH, DIR_GRAPH, MULTI_GRAPH, LAYER_GRAPH};
private:
    vector<Node<T>> nodes;
    set<Edge<T>> edges;
    vector<Edge<T>> multiEdges;
    graphType type;
    int numLevels;

public:
    Graph(){
        nodes.resize(0);
//        edges.resize(0);
    }
    void addNode(Node<T> node){
        nodes.push_back(node);
    }
    void addNodes(vector<Node<T>> vecNodes){
        for(Node<T> node : vecNodes){
            nodes.push_back(node);
        }
    }
    bool addEdge(Node<T> &nodeA, Node<T> &nodeB){
        if(!nodeA.addNeighbor(nodeB) && type != MULTI_GRAPH){
//            std::cerr << "First node at " << &nodeA << " with value " << nodeA.getValue()
//                      << " Is already a neighbor of the other provided node." << endl;
            return false;
        }
        if(!nodeB.addNeighbor(nodeA) && type != MULTI_GRAPH){
//            std::cerr << "First node at " << &nodeB << " with value " << nodeB.getValue()
//                      << " Is already a neighbor of the other provided node." << endl;
            return false;
        }

        Edge<T> newEdge = Edge<T>(nodeA, nodeB);

        bool added = edges.insert(newEdge).second;
        if(!added && type == MULTI_GRAPH){
            multiEdges.push_back(newEdge);
            return true;
        }
        else return added;


    }


    /**
     *
     * @param r                     Random object used to generate the random graph
     * @param dims                  Dimensions of each layer starting with top layer. Contains X and Y respectively for each layer.
     *                              Should be size = 2*layers
     * @param layers                Number of layers in the multi layer graph
     * @param minMaxchildren        Minimum and Maximum number of children per layer. Come in pairs <Min_top, Max_top, Min_2nd, Max_2nd> etc
     *                              Should be size = 2*(layers-1)
     * @param probNeighbor          Probability of adjacent nodes being declared neighbors of any given node on each layer <Prob_top, Prob_2nd> etc
     *                              Should be size = layers
     *
     * @return                      A random multi-tiered graph based on specifications given. The top layer is generated as a grid
     *                              and the following layers have points randomly placed within the region given
     *
     */
    static Graph<T>* genRandomMultiLayerGraph(Rand &r, const vector<int> &dims, int layers, const vector<int> &minMaxchildren, vector<float> probNeighbor = {-1.0}){
        //resize neighbor prob to be correct size if using the default vector
        if(probNeighbor.size() == 1 && probNeighbor[0] == -1.0){
            probNeighbor.resize(layers);
            if(probNeighbor.size() != 0) std::fill(probNeighbor.begin(), probNeighbor.end(), 1.0);
        }

        //Size checks
        if(dims.size() != layers*2) throw Ex(to_string(dims.size()) + " of " + to_string(layers*2) + " dims given for " + to_string(layers) + " layers.");
        if(minMaxchildren.size() != (layers-1)*2) throw Ex(to_string(minMaxchildren.size()) + " of " + to_string((layers-1)*2) + " child bounds given for " + to_string(layers) + " layers.");
        if(probNeighbor.size() != layers) throw Ex(to_string(probNeighbor.size()) + " of " + to_string(layers) + " neighbor probability values given for " + to_string(layers) + " layers.");
        Graph<T>* g = new Graph();
        g->type = graphType::LAYER_GRAPH;
        g->numLevels = layers;
        g->nodes.resize(0);
        //build the top layer
        for(int x = 0; x < dims[0]; x++){
            for(int y = 0; y < dims[1]; y++){
                Node<T> n(x, y, layers, x + (y*dims[0]));
                g->addNode(n);
            }
        }
        //Connect top Layer
        for(int n1 = 0; n1 < g->getNumNodes(); n1++) {
            for(int n2 = 0; n2 < g->getNumNodes(); n2++) {
                int diff = abs(g->getNodes()[n1].getX() - g->getNodes()[n2].getX()) + abs(g->getNodes()[n1].getY() - g->getNodes()[n2].getY());
                if (diff == -1 || diff == 1) {
                    if (r.uniform() < probNeighbor[0]) {
                        g->addEdge(g->getNodes()[n1], g->getNodes()[n2]);
                    }
                }
            }
        }


        //build child layers -- 2nd layer is built independently from further layers
        if(layers > 1){
            int baseID = 0;

            for(int n = 0; n < g->getNumNodes(); n++){
                Graph<T> *childGraph = genRandomGraph(r, dims[2], dims[3], layers-1, minMaxchildren[0], minMaxchildren[1], probNeighbor[1],baseID);
                g->nodes[n].setChildGraph(childGraph);
            }
        }

        return g;
    }

    static Graph<T>* genRandomGraph(Rand &r, int x_dim, int y_dim, int layer, int min_nodes, int max_nodes, float neighbor_prob, int &baseID){
        int numNodes = (max_nodes-min_nodes == 0 ? max_nodes : r.next(max_nodes-min_nodes) + min_nodes);
        Graph<T>* g =  new Graph();
        g->type = graphType::LAYER_GRAPH;
        g->nodes.resize(0);
        for(int c = 0; c < numNodes; c++){
            Node<T> n(r.next(x_dim), r.next(y_dim), layer, baseID);
            baseID++;
            g->addNode(n);
        }
        //Connect Edges
        for(int n1 = 0; n1 < g->getNumNodes(); n1++){
            for(int n2 = 0; n2 < g->getNumNodes();n2++){
                if(n1 != n2){
                    if(r.uniform() < neighbor_prob){}
                    g->addEdge(g->getNodes()[n1], g->getNodes()[n2]);
                }
            }
        }

        return g;
    }


    static pair<Graph<T>, Graph<T>> genRandomMLandMatchingSLGraph(){

    }


    size_t getNumEdges(){
        return edges.size();
    }

    size_t getNumNodes(){
        return nodes.size();
    }

    vector<Node<T>> &getNodes() {
        return nodes;
    }

    const vector<Node<T>> &getNodesAtLevel(int level) const{

    }
    const vector<Edge<T>> &getEdges() const {
        return edges;
    }


    void printNodes(){
        for(Node<T> node : nodes){
            node.print();
        }
    }

    void printEdges(){
        cout << edges.size() << endl;
        for(Edge<T> edge: edges){
            edge.printEdge();
        }
    }



};


#endif //GRAPHLIBRARY_GRAPH_H
