import networkx as nx
from math import sqrt
# import random
from numpy import random
class Node:

    def __init__(self, x_loc=0, y_loc=0, node_id=-1, cost=float("inf"), heuristic=0.0, g=nx.Graph(), threadCount=1):
        self.x_loc = x_loc
        self.y_loc = y_loc
        self.node_id = node_id
        self.startPoint = None
        self.endPoint = dict()
        self.ep = None
        self.cost = cost
        self.parent = None
        self.heuristic = heuristic
        self.requests = 0
        # self.cost = [float("inf") for i in range(threadCount)]
        # self.heuristic = [0 for i in range(threadCount)]
        # self.startPoint = [None for i in range(threadCount)]
        # self.endPoint = [{} for i in range(threadCount)]
        # self.ep = [None for i in range(threadCount)]
        # self.parent = [None for i in range(threadCount)]
        # self.requests = [0 for i in range(threadCount)]


        self.g = g
        # -1x, +1x, -1y, +1y
        self.forcedPoints = [None, None, None, None]
        # left, right, bottom, top
        self.reconciled = [False, False, False, False]
        self.leftOverlap = []
        self.rightOverlap = []
        self.topOverlap = []
        self.bottomOverlap = []
        self.shortestPaths = None
        self.threadCount = threadCount

        self.sortCost = float("inf")
        self.sortHeuristic = 0


    def __getitem__(self, item):
        self.x_loc = item.x_loc
        self.y_loc = item.y_loc
        self.startPoint = item.startPoint
        self.endPoint = item.endPoint
        self.node_id = item.node_id
        self.cost = item.cost
        self.heuristic = item.heuristic
        self.g = item.g
        # self = item

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    # def __lt__(self, other):
    #     return (self.sortCost + self.sortHeuristic) < (other.sortCost + other.sortHeuristic)

    def printNode(self):
        print "Node Id: " + str(self.node_id)
        print "Y_loc: " + str(self.y_loc)
        print "X_loc: " + str(self.x_loc)
        print "Start Point: " + str(self.startPoint)
        # print "End Point: " + str(self.endPoint)
        print "Cost: " + str(self.cost)
        print "Heuristic: " + str(self.heuristic)
        print ""

    def printLoc(self):
        print "Node Id: " + str(self.node_id)
        print "(X,Y): (" + str(self.x_loc) + ", " + str(self.y_loc) + ")"
        # print "Y_loc: " + str(self.y_loc)
        # print "X_loc: " + str(self.x_loc)

    def printId(self):
        print "Node Id: " + str(self.node_id)



    def printEdgesById(self):
        edge_str = "["
        first = True
        for edge in self.g.edges():
            if not first:
                edge_str += ", "
            first = False
            edge_str += "(" + str(edge[0].node_id) + ", " + str(edge[1].node_id) + ")"
        edge_str += "]"
        print edge_str



    # def getCost(self, node):
    #     if self is node:
    #         return self.cost
    #     else:
    #         self.requests += 1
    #         return node.getCost(node)



    def calcDistance(self, n1, n2):
        x2 = (n1.x_loc - n2.x_loc) * (n1.x_loc - n2.x_loc)
        y2 = (n1.y_loc - n2.y_loc) * (n1.y_loc - n2.y_loc)
        return sqrt(x2 + y2)

    def genRandomCost(self):
        self.cost = random.randint(0,150)


    def getDistToGoal(self, node, Cg):
        if self is node:
            return self.calcDistance(self, Cg)
        else:
            self.requests += 1
            return node.getDistToGoal(node, Cg)




class CeilingNode(Node):
    children = []
    def findOverlap(self, current, neighbor):
        if self is current:
            if self.x_loc - neighbor.x_loc == 1:
                ret = [self.leftOverlap[0], self.requestRightOverlap(neighbor)]
            elif self.x_loc - neighbor.x_loc == -1:
                ret = [self.rightOverlap[0], self.requestLeftOverlap(neighbor)]
            elif self.y_loc - neighbor.y_loc == 1:
                ret = [self.topOverlap[0], self.requestBottomOverlap(neighbor)]
            elif self.y_loc - neighbor.y_loc == -1:
                ret = [self.bottomOverlap[0], self.requestTopOverlap(neighbor)]
            else:
                ret = None
            # all require a request of their neighbor's overlap point, so add request here
            # self.requests += 1
            return ret
        else:
            self.requests += 1
            return current.findOverlap(current, neighbor)

    def SetEdgePoints(self):
        nodes = self.g.nodes()
        for idx, val in enumerate(self.forcedPoints):
            if self.forcedPoints[idx] is None:
                self.forcedPoints[idx] = random.choice(nodes)
                # self.forcedPoints[idx] = random.sample(nodes, 1)[0]
        for node in nodes:
            # min x, left neighbor
            if self.forcedPoints[0].x_loc > node.x_loc:
                self.forcedPoints[0] = node
            # max x, right neighbor
            if self.forcedPoints[1].x_loc < node.x_loc:
                self.forcedPoints[1] = node
            # min y, top neighbor
            if self.forcedPoints[2].y_loc > node.y_loc:
                self.forcedPoints[2] = node
            # max y, bottom neighbor
            if self.forcedPoints[3].y_loc < node.y_loc:
                self.forcedPoints[3] = node

    def rebuildPath(self, Cg):
        length = 0
        numCameras = 0
        # interior if statement is to check path length. Each parent assignment is effectively an edge
        # so if the value is None after assignment, there is no edge so we don't increment
        currentNode = Cg
        pathStack = []
        end = None
        while currentNode is not None:
            if currentNode is Cg:
                end = Cg.ep
            # If the start/goal point isn't the overlap point, calc path length
            # this should only apply for the goal camera or start camera

            if end is not currentNode.startPoint:
                p = self.getShortestPath(currentNode, currentNode.startPoint, end)
                # greedy breaks if it can't reach the goal here because greedy. So this statement is to "fix" greedy
                if p == float('inf'):
                    return None
                pathStack = [p] + pathStack
                l_cam = 0
                for idx in range(len(p)-1):
                    n1 = p[idx]
                    n2 = p[idx+1]
                    l = self.calcDistance(n1,n2)
                    l_cam += l
                    length += l
            numCameras += 1
            if currentNode.parent is not None:
                end = currentNode.parent.getEndpoint(currentNode)

            currentNode = currentNode.parent
        return length

    def getShortestPath(self, node, start, end):
        if self is node:
            try:
                sp = self.shortestPaths[start][end]
            except:
                sp = float('inf')
            return sp
        else:
            self.requests += 1
            return node.getShortestPath(node, start, end)

    def requestRightOverlap(self, n):
        return n.rightOverlap[0]

    def requestLeftOverlap(self, n):
        return n.leftOverlap[0]

    def requestTopOverlap(self, n):
        return n.topOverlap[0]

    def requestBottomOverlap(self, n):
        return n.bottomOverlap[0]

    def getMinX(self):
        return self.forcedPoints[0].x_loc
    def getMaxX(self):
        return self.forcedPoints[1].x_loc
    def getMinY(self):
        return self.forcedPoints[2].y_loc
    def getMaxY(self):
        return self.forcedPoints[3].y_loc

    def getEndpoint(self, neighbor):
        return self.endPoint[neighbor]

    def minHeuristicNode(self, n1, n2, c_goal):
        if n1 is None:
            return n2
        elif n2 is None:
            return n1

        if n1 is self:
            hn1 = self.getHeuristic(c_goal)
        else:
            self.requests += 1
            hn1 = self.requestHeuristic(n1, c_goal)
        if n2 is self:
            hn2 = self.getHeuristic(c_goal)
        else:
            self.requests += 1
            hn2 = self.requestHeuristic(n2, c_goal)
        if hn1 < hn2:
            return n1
        return n2


    def getHeuristic(self, Cg):
        x2 = (self.x_loc - Cg.x_loc) * (self.x_loc - Cg.x_loc)
        y2 = (self.y_loc - Cg.y_loc) * (self.y_loc - Cg.y_loc)
        return sqrt(x2 + y2)

    def requestHeuristic(self, n, Cg):
        return n.getHeuristic(Cg)

    def reconcileForcedPoints(self, neighbors):
        for neighbor in neighbors:
            # right neighbor
            if self.x_loc - neighbor.x_loc == -1:
                if neighbor.reconciled[0]:
                    continue
                self.reconciled[1] = True
                neighbor.reconciled[0] = True
                c = GroundNode(x_loc=self.getMaxX(), y_loc=neighbor.forcedPoints[0].y_loc, node_id=len(self.g.nodes())+1)
                self.g.add_node(c)
                self.rightOverlap.append(c)
                neighbor.leftOverlap.append(neighbor.forcedPoints[0])
            # left neighbor
            elif self.x_loc - neighbor.x_loc == 1:
                if neighbor.reconciled[1]:
                    continue
                self.reconciled[0] = True
                neighbor.reconciled[1] = True
                c = GroundNode(x_loc=self.getMinX(), y_loc=neighbor.forcedPoints[1].y_loc, node_id=len(self.g.nodes())+1)
                self.g.add_node(c)
                self.leftOverlap.append(c)
                neighbor.rightOverlap.append(neighbor.forcedPoints[1])
            # bottom neighbor
            elif self.y_loc - neighbor.y_loc == -1:
                if neighbor.reconciled[3]:
                    continue
                self.reconciled[2] = True
                neighbor.reconciled[3] = True
                c = GroundNode(x_loc=neighbor.forcedPoints[2].x_loc, y_loc=self.getMaxY(), node_id=len(self.g.nodes())+1)
                self.g.add_node(c)
                self.bottomOverlap.append(c)
                neighbor.topOverlap.append(neighbor.forcedPoints[2])
            # top neighbor
            elif self.y_loc - neighbor.y_loc == 1:
                if neighbor.reconciled[2]:
                    continue
                self.reconciled[3] = True
                neighbor.reconciled[2] = True
                c = GroundNode(x_loc=neighbor.forcedPoints[3].x_loc, y_loc=self.getMinY(), node_id=len(self.g.nodes())+1)
                self.g.add_node(c)
                self.bottomOverlap.append(c)
                neighbor.topOverlap.append(neighbor.forcedPoints[3])

    def allPairsShortestPaths(self):
        self.shortestPaths = dict(nx.all_pairs_dijkstra_path(self.g))

    def addRandomEdgesInChild(self, prob):
        for n1 in self.g.nodes():
            for n2 in self.g.nodes():
                if n1 is not n2 and not self.g.has_edge(n1, n2):
                    if random.random() < prob:
                        self.g.add_edge(n1, n2, weight=self.calcDistance(n1, n2))

    def getPathCost(self, currentNode, neighbor):
        length = 0
        if self is currentNode:
            overlapPoints = self.findOverlap(currentNode, neighbor)
            if overlapPoints[0] is not self.startPoint:
                p = self.getShortestPath(self, self.startPoint, overlapPoints[0])
                if p == float('inf'):
                    return float('inf')
                for idx in range(len(p) - 1):
                    n1 = p[idx]
                    n2 = p[idx + 1]
                    length += self.calcDistance(n1, n2)
        else:
            self.requests += 1
            return currentNode.getPathCost(currentNode, neighbor)
        return length

    def getNeighborsInfo(self, neighbors, currentNode, Cg):
        if self is currentNode:
            updatedValues = list()
            for neighbor in neighbors:
                potentialCost = currentNode.cost + currentNode.getPathCost(currentNode, neighbor)
            #     # add the cost of the path inside Cg to potential cost
                if neighbor is Cg:
                    length = 0
                    overlapPoints = currentNode.findOverlap(currentNode, neighbor)
                    if Cg.ep is not overlapPoints[1]:
                        p = currentNode.getShortestPath(Cg, overlapPoints[1], Cg.ep)
                        if p == float('inf'):
                            length = float('inf')
                        else:
                            for idx in range(len(p) - 1):
                                n1 = p[idx]
                                n2 = p[idx + 1]
                                length += currentNode.calcDistance(n1, n2)
                    potentialCost += length

                if potentialCost < neighbor.cost:
                    overlapPoints = currentNode.findOverlap(currentNode, neighbor)
                    currentNode.endPoint[neighbor] = overlapPoints[0]
                    neighbor.startPoint = overlapPoints[1]
                    neighbor.parent = currentNode
                    neighbor.cost = potentialCost
                    h = currentNode.getDistToGoal(neighbor, Cg)
                    neighbor.heuristic = h

                updatedValues.append([neighbor.cost, neighbor.heuristic, neighbor.startPoint, neighbor.parent, currentNode.endPoint[neighbor]])
            return updatedValues
        else:
            self.requests += 1
            return currentNode.getNeighborsInfo(neighbors, currentNode, Cg)


class GroundNode(Node):
    parents = []