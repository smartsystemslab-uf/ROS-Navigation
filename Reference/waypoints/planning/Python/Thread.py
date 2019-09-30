from Node import *

import threading
from numpy import random
import time
import networkx as nx
from Queue import PriorityQueue


class PathThread (threading.Thread):

    # lock = threading.Lock()
    # a = 5

    def __init__(self, tId, start, end, g, lock):
        threading.Thread.__init__(self)
        self.threadID = tId
        self.startNode = start
        self.endNode = end
        self.g = g
        self.pathLength = 0
        self.runtime = 0
        self.requestTotal = 0
        self.lock = lock
        self.pqueue = PriorityQueue()


        self.cost = [float("inf") for i in range(len(self.g.nodes()))]
        self.heuristic = [0 for i in range(len(self.g.nodes()))]
        self.startPoint = [None for i in range(len(self.g.nodes()))]
        self.endPoint = [{} for i in range(len(self.g.nodes()))]
        self.ep = [None for i in range(len(self.g.nodes()))]
        self.parent = [None for i in range(len(self.g.nodes()))]
        self.requests = [0 for i in range(len(self.g.nodes()))]
        # self.locks = locks

        # initialize the lists defined above with correct values for each node
        self.setInstanceValues()


    def run(self):
        # self.lock.acquire()
        # self.setNodes()
        start = time.time()
        self.Astar2()
        end = time.time()
        # self.lock.release()
        self.runtime = end-start

        # self.queueTest(a)
        # self.lockTest(5)
    #     for all nodes in G, total requests at index threadId

    def Astar2(self):
        pqueue = PriorityQueue()
        closedSet = set()
        openset = set()

        found = False
        openset.add(self.startNode)
        self.lock.acquire()
        # self.lockCriticalNodes(openset)
        self.setNodes()
        # self.startNode.cost = 0
        self.cost[self.startNode.node_id] = 0
        pqueue.put(self.startNode)
        # self.releaseCriticalNodes(openset)
        self.lock.release()

        while not pqueue.empty():
            self.lock.acquire()
            # self.lockCriticalNodes(openset)
            self.setNodes(openset)
            currentNode = pqueue.get()
            # self.releaseCriticalNodes(openset)
            self.lock.release()

            if currentNode is self.endNode and currentNode.cost != float('inf'):
                found = True
                closedSet.add(currentNode)
                self.lock.acquire()
                # self.lockCriticalNodes(closedSet)
                self.setNodes(closedSet)
                self.pathLength = self.startNode.rebuildPath(currentNode)
                # self.releaseCriticalNodes(closedSet)
                self.lock.release()
            # self.lock.acquire()
            # self.setNodes()
            # self.startNode.requests += 1
            self.requests[self.startNode.node_id] += 1
            # self.lock.release()
            neighbors = list()
            # only operate on neighbors not in the closed set
            for neighbor in list(nx.neighbors(self.g, currentNode)):
                if neighbor in closedSet:
                    continue
                if neighbor not in openset:
                    self.lock.acquire()
                    openset.add(neighbor)
                    # self.lockCriticalNodes(openset)
                    self.setNodes(openset)
                    pqueue.put(neighbor)
                    # self.releaseCriticalNodes(openset)
                    self.lock.release()
                neighbors.append(neighbor)

            self.lock.acquire()
            # self.lockCriticalNodes(openset)
            self.setNodes(openset)
            self.startNode.getNeighborsInfo(neighbors, currentNode, self.endNode)
            self.setInstanceValues(neighbors)
            # self.releaseCriticalNodes(openset)
            self.lock.release()
            closedSet.add(currentNode)
            openset.remove(currentNode)

        if not found:
            self.pathLength = None

    def setNodes(self, set=None):
        if set is None:
            for node in self.g.nodes():
                index = node.node_id
                node.cost = self.cost[index]
                node.heuristic = self.heuristic[index]
                node.startPoint = self.startPoint[index]
                node.endPoint = self.endPoint[index]
                node.ep = self.ep[index]
                node.requests = self.requests[index]
                node.parent = self.parent[index]
        else:
            for node in list(set):
                index = node.node_id
                node.cost = self.cost[index]
                node.heuristic = self.heuristic[index]
                node.startPoint = self.startPoint[index]
                node.endPoint = self.endPoint[index]
                node.ep = self.ep[index]
                node.requests = self.requests[index]
                node.parent = self.parent[index]

    # def lockTest(self, n):
    #     self.lock.acquire()
    #     for i in range(n):
    #         print "printing from thread %d, iteration %d" % (self.threadID, i)
    #     self.lock.release()

    def setTest(self):
        self.startNode.sortCost = 3
        self.startNode.sortHeuristic = 4

    # def queueTest(self, nodeSet):
    #     # pqueue = PriorityQueue()
    #     self.lock.acquire()
    #     # if change:
    #     #     node = nodeSet[len(nodeSet)-1]
    #     #     node.sortCost = 1
    #     #     node.sortHeuristic = 1
    #     for node in nodeSet:
    #         self.pqueue.put(node)
    #     self.lock.release()
    #     # self.startNode.sortCost = 1
    #     # self.startNode.sortHeuristic = 1

    def setInstanceValues(self, set=None):
        if set is None:
            for node in self.g.nodes():
                index = node.node_id
                self.cost[index] = node.cost
                self.heuristic[index] = node.heuristic
                self.startPoint[index] = node.startPoint
                self.endPoint[index] = node.endPoint
                self.ep[index] = node.ep
                self.requests[index] = node.requests
                self.parent[index] = node.parent
        else:
            for node in list(set):
                index = node.node_id
                self.cost[index] = node.cost
                self.heuristic[index] = node.heuristic
                self.startPoint[index] = node.startPoint
                self.endPoint[index] = node.endPoint
                self.ep[index] = node.ep
                self.requests[index] = node.requests
                self.parent[index] = node.parent


    def lockCriticalNodes(self, set):
        for node in set:
            self.locks[node.node_id].acquire()

    def releaseCriticalNodes(self, set):
        for node in set:
            self.locks[node.node_id].release()