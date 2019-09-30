from Node import *
from Thread import PathThread

from numpy import random
import time
import networkx as nx
import thread
from Queue import PriorityQueue
import threading


# random.seed(75)





def buildGraph(ceilingXRange, ceilingYRange, minChildren, maxChildren, probChild, probNeighbor=1.0):
    ceilingCams = []
    # make sure to init the graph here otherwise they all share the same graph
    for x in range(ceilingXRange):
        for y in range(ceilingYRange):
            ceilingCams.append(CeilingNode(x_loc=x, y_loc=y, node_id=y * ceilingXRange + x, g=nx.Graph()))

    g = nx.Graph()
    g.add_nodes_from(ceilingCams)

    # if it's one away, they're neighbors
    for cam in ceilingCams:
        for cam2 in ceilingCams:
            diff = abs((cam.x_loc - cam2.x_loc)) + abs((cam.y_loc - cam2.y_loc))
            if diff == 1 or diff == -1:
                if random.random() < probNeighbor:
                    cam.endPoint[cam2] = None
                    cam2.endPoint[cam] = None
                    g.add_edge(cam, cam2)

    # randomly generate children or read from adjlist
    for cam in ceilingCams:
        numChildren = random.randint(minChildren, maxChildren)
        for c in range(numChildren):
            cam.g.add_node(GroundNode(x_loc=random.randint(0,100), y_loc=random.randint(0,100), node_id=c))
        cam.SetEdgePoints()

    for cam in ceilingCams:
        cam.reconcileForcedPoints(g.neighbors(cam))
        cam.addRandomEdgesInChild(probChild)
        cam.allPairsShortestPaths()

    return g

def Astar(g, Cs_c, Cg_c):
    pqueue = PriorityQueue()
    closedSet = set()
    openset = set()
    Cs_c.cost = 0
    pqueue.put(Cs_c)
    openset.add(Cs_c)
    while not pqueue.empty():
        currentNode = pqueue.get()

        if currentNode is Cg_c and currentNode.cost != float('inf'):
            return Cs_c.rebuildPath(currentNode)
        Cs_c.requests += 1
        for neighbor in nx.neighbors(g, currentNode):
            if neighbor in closedSet:
                continue
            if neighbor not in openset:
                pqueue.put(neighbor)
                openset.add(neighbor)
            potentialCost = currentNode.cost + Cs_c.getPathCost(currentNode, neighbor)

            # add the cost of the path inside Cg to potential cost
            if neighbor is Cg_c:
                length = 0
                overlapPoints = Cs_c.findOverlap(currentNode, neighbor)
                if Cg_c.ep is not overlapPoints[1]:
                    p = Cs_c.getShortestPath(Cg_c, overlapPoints[1], Cg_c.ep)
                    if p == float('inf'):
                        length = float('inf')
                    else:
                        for idx in range(len(p) - 1):
                            n1 = p[idx]
                            n2 = p[idx + 1]
                            length += Cs_c.calcDistance(n1, n2)
                potentialCost += length

            if potentialCost < neighbor.cost:
                overlapPoints = Cs_c.findOverlap(currentNode, neighbor)
                currentNode.endPoint[neighbor] = overlapPoints[0]
                neighbor.startPoint = overlapPoints[1]
                neighbor.parent = currentNode
                neighbor.cost = potentialCost
                h = Cs_c.getDistToGoal(neighbor, Cg_c)
                neighbor.heuristic = h
        closedSet.add(currentNode)
        openset.remove(currentNode)
    # print "Failure"


def AStarCascade(g, Cs_c, Cg_c):
    print "Failure"


def AStarOwnCalc(g, Cs_c, Cg_c):
    pqueue = PriorityQueue()
    closedSet = set()
    openset = set()
    Cs_c.cost = 0
    pqueue.put(Cs_c)
    openset.add(Cs_c)
    while not pqueue.empty():
        currentNode = pqueue.get()
        if currentNode is Cg_c and currentNode.cost != float('inf'):
            return Cs_c.rebuildPath(currentNode)
        Cs_c.requests += 1
        neighbors = list()
        # only operate on neighbors not in the closed set
        for neighbor in list(nx.neighbors(g, currentNode)):
            if neighbor in closedSet:
                continue
            if neighbor not in openset:
                pqueue.put(neighbor)
                openset.add(neighbor)
            neighbors.append(neighbor)

        Cs_c.getNeighborsInfo(neighbors, currentNode, Cg_c)
        closedSet.add(currentNode)
        openset.remove(currentNode)

def GreedySearch(g, Cs, Cg):
    currentNode = Cs
    Cs.cost = 0
    while currentNode is not None:
        if currentNode is Cg:
            if currentNode.cost != float('inf'):
                return None
            return Cs.rebuildPath(currentNode)
        nextNode = None
        # add request for neighbors
        Cs.requests += 1
        for neighbor in nx.neighbors(g, currentNode):
            nextNode = Cs.minHeuristicNode(neighbor, nextNode, Cg)
        nextNode.parent = currentNode
        overlapPoints = Cs.findOverlap(currentNode, nextNode)
        # print "Setting currentnode Endpoint to " + str(overlapPoints[0])
        currentNode.endPoint[nextNode] = overlapPoints[0]
        # print "Setting neighbor startpoint to " + str(overlapPoints[1])
        nextNode.startPoint = overlapPoints[1]
        currentNode = nextNode
    print "Path not found"
    return None

def calcCost(p):
    length = 0
    for path in p:
        # print path
        for idx in range(len(path) - 1):
            n1 = path[idx]
            n2 = path[idx + 1]
            length += n1.calcDistance(n1, n2)
    # print length
    return length


def batchNormalizeAndWrite(fn1, fn2, d1, d2, norm, normd1=False):
    if len(d1) != len(d2):
        print "mismatch"
        quit(-1)

    s1 = ""
    s2 = ""
    for idx,val in enumerate(d1):
        # print idx
        if norm[idx]:
            if normd1:
                val = d1[idx]
            else:
                val = d2[idx]
        else:
            val = 1
        d1[idx] /= float(val)
        d2[idx] /= float(val)
        s1 += str(float(d1[idx])) + "\t"
        s2 += str(float(d2[idx])) + "\t"
    s1 += "\n"
    s2 += "\n"
    f = open(fn1, 'a')
    f2 = open(fn2, 'a')
    f.write(s1)
    f2.write(s2)
    f.close()
    f2.close()


def normalizeAndWrite(f, d):
    s = ""
    for el in d:
        s += str(el) + "\t\t"
    # s = str(d[0]) + "\t\t" + str(d[1]) + "\t\t" + str(d[2]) + "\t\t"
    f.write(s)

def resetNodes(g):
    for node in g.nodes():
        node.cost = float('inf')
        node.heuristic = 0.0
        node.parent = None
        node.requests = 0
        # node.startPoint = None
        # node.endPoint = dict()
        # node.ep = None


def MultithreadedAStarCompare():
    connectivityChild = [.25, .5, .75]
    gridSize = [5, 10, 20, 50]
    normalized = [True, False]
    for norm in normalized:
        for gs in gridSize:
            for conn in connectivityChild:

                xDim = gs
                yDim = xDim
                minChildren = 5
                maxChildren = 10
                probChildConnect = conn
                probCeilConnect = .8


                filenamePrefix = str(xDim) + "x" + str(yDim) + "_" + str(probChildConnect).split('.')[1]
                if norm:
                    dirExtension = "_normalized"
                else:
                    dirExtension = "_unnormalized"
                dirName = str(filenamePrefix).split("_")[0] + dirExtension + "/"
                # print filenamePrefix
                fn1 = dirName+filenamePrefix + "_mt_time.txt"
                astF = open(fn1, 'w')
                astF.write("Main Thread\tMulti Thread\n")
                astF.close()



                lock = threading.Lock()

                print "Running a random path on a random graph of size %d by %d with connectivity %f to check thread call of A*" % (xDim, yDim, probChildConnect)

                g = buildGraph(xDim, yDim, minChildren, maxChildren, probChildConnect, probCeilConnect)

                # print "Graph built"
                pathCount = 2


                # locks = [threading.Lock() for i in range(len(g.nodes()))]
                MainThreadLengths = []

                threads = []

                mainThreadTime = 0

                for i in range(pathCount):
                    random.seed(i)
                    Cs = random.choice(g.nodes())
                    while len(Cs.g.edges()) == 0:
                        Cs = random.choice(g.nodes())
                    Cg = Cs
                    while Cg is Cs or len(Cg.g.edges()) == 0:
                        Cg = random.choice(g.nodes())
                    Ps = random.choice(Cs.g.nodes())
                    Pe = random.choice(Cg.g.nodes())
                    Cs.startPoint = Ps
                    Cg.ep = Pe

                    # Cs_a.append(Cs)
                    # Cg_a.append(Cg)
                    thread = PathThread(i, Cs, Cg, g, lock)
                    threads.append(thread)
                    start_time = time.time()
                    astOCC = AStarOwnCalc(g, Cs, Cg)
                    end_time = time.time()
                    mainThreadTime += end_time-start_time
                    MainThreadLengths.append(astOCC)
                    resetNodes(g)
                    # print astOCC

                # print "Main threads done"

                # Time from threads starting to all completing
                s = time.time()
                for t in threads:
                    t.start()

                for t in threads:
                    t.join()
                e = time.time()
                multithreadTime = e-s

                for idx,val in enumerate(threads):
                    if MainThreadLengths[idx] != threads[idx].pathLength:
                        print "diff"
                        print MainThreadLengths[idx]
                        print threads[idx].pathLength
                resetNodes(g)

                print "Total time taken to run %d paths on Main thread: %f" % (pathCount, mainThreadTime)
                print "Total time taken to run %d paths with multithreading: %f" % (pathCount, multithreadTime)

                astF = open(fn1, 'a')
                astF.write("%f\t%f" % (mainThreadTime, multithreadTime))



def ExhaustiveComparison():
    connectivityChild = [.25, .5, .75]
    gridSize = [5, 10, 20, 50, 100]
    normalized = [True]
    for norm in normalized:
        for gs in gridSize:
            for conn in connectivityChild:

                xDim = gs
                yDim = xDim
                minChildren = 5
                maxChildren = 10
                probChildConnect = conn
                probCeilConnect = 1

                filenamePrefix = str(xDim) + "x" + str(yDim) + "_" + str(probChildConnect).split('.')[1]
                if norm:
                    dirExtension = "_normalized"
                else:
                    dirExtension = "_unnormalized"
                dirName = str(filenamePrefix).split("_")[0] + dirExtension + "/"
                # print filenamePrefix
                fn1 = dirName + filenamePrefix + "_grd.txt"
                fn2 = dirName + filenamePrefix + "_ast2_grd.txt"
                astF = open(fn1, 'w')
                astF.write("Cost\tRequests\tRuntime\n")
                ast2F = open(fn2, 'w')
                ast2F.write("Cost\tRequests\tRuntime\n")
                astF.close()
                ast2F.close()

                g = buildGraph(xDim, yDim, minChildren, maxChildren, probChildConnect, probCeilConnect)
                #
                # grdCcount = 0
                # grdRcount = 0
                # grdTcount = 0
                astCtotal = 0
                astRtotal = 0
                astTtotal = 0

                grdCtotal = 0
                grdRtotal = 0
                grdTtotal = 0

                # astOCCcount = 0
                # astOCRcount = 0
                # astOCTcount = 0
                astOCCtotal = 0
                astOCRtotal = 0
                astOCTtotal = 0

                iterations = 1000
                for i in range(iterations):

                    random.seed(i)
                    # if i%100 == 0:
                    print str(i) + " " + filenamePrefix

                    random.seed(i)
                    Cs_c = random.choice(g.nodes())
                    while len(Cs_c.g.edges()) == 0:
                        Cs_c = random.choice(g.nodes())

                    Cg_c = Cs_c
                    while Cg_c is Cs_c or len(Cg_c.g.edges()) == 0:
                        Cg_c = random.choice(g.nodes())
                    Ps_c = random.choice(Cs_c.g.nodes())
                    Pe_c = random.choice(Cg_c.g.nodes())
                    Cs_c.startPoint = Ps_c
                    Cg_c.ep = Pe_c

                    # Greedy pathing and data collection
                    start_time = time.time()
                    grdC = GreedySearch(g, Cs_c, Cg_c)
                    # print Astar(g, Cs_c, Cg_c)
                    end_time = time.time()
                    grdT = end_time - start_time
                    grdR = Cs_c.requests
                    grdTotalRLocal = 0
                    if grdC is not None and grdC is not 0:
                        # astRuntimes.append(end_time - start_time)
                        # astPaths.append(astC)
                        # astRequests.append(Cs_c.requests)
                        for node in g.nodes():
                            grdTotalRLocal += node.requests
                            # astRtotal += node.requests

                        # astCtotal += astC
                        # astTtotal += astT
                        # astRtotal += astTotalRLocal
                    else:
                        # failureAST += 1
                        i -= 1
                        # print "Cont 1"
                        resetNodes(g)
                        continue

                    # # A* pathing and data collection
                    # start_time = time.time()
                    # astC = Astar(g, Cs_c, Cg_c)
                    # # print Astar(g, Cs_c, Cg_c)
                    # end_time = time.time()
                    # astT = end_time - start_time
                    # astR = Cs_c.requests
                    # astTotalRLocal = 0
                    # if astC is not None:
                    #     # astRuntimes.append(end_time - start_time)
                    #     # astPaths.append(astC)
                    #     # astRequests.append(Cs_c.requests)
                    #     for node in g.nodes():
                    #         astTotalRLocal += node.requests
                    #         # astRtotal += node.requests
                    #
                    #     # astCtotal += astC
                    #     # astTtotal += astT
                    #     # astRtotal += astTotalRLocal
                    # else:
                    #     # failureAST += 1
                    #     i -= 1
                    #     # print "Cont 1"
                    #     resetNodes(g)
                    #     continue


                    Cs_c.requests = 0

                    # Reset data in the nodes so they don't interfere with subsequent searches. I.e costs/parents/etc
                    resetNodes(g)

                    # print ""
                    start_time = time.time()
                    astOCC = AStarOwnCalc(g, Cs_c, Cg_c)
                    end_time = time.time()
                    astOCT = end_time - start_time
                    astOCTotalR = 0
                    if astOCC is not None:
                        # print astOCC
                        for node in g.nodes():
                            astOCTotalR += node.requests

                        # astOCCtotal += astOCC
                        # astOCRtotal += astOCTotalR
                        # astOCTtotal += astOCT
                    else:
                        i -= 1
                        # print "cont 2"
                        resetNodes(g)
                        continue
                        # failureAST2 += 1
                    resetNodes(g)
                    # astCtotal += astC
                    # astTtotal += astT
                    # astRtotal += astTotalRLocal
                    grdCtotal += grdC
                    grdTtotal += grdT
                    grdRtotal += grdTotalRLocal
                    astOCCtotal += astOCC
                    astOCRtotal += astOCTotalR
                    astOCTtotal += astOCT

                    # print "Astar cost better: " + str(astCcount) + " with average " + str(astCtotal/float(iterations))
                    # print "Astar v2 cost better or equal: " + str(astOCCcount) + " with average " + str(astOCCtotal/float(iterations))
                    # print ""
                    # print "Astar requests lower: " + str(astRcount) + " with average " + str(astRtotal/float(iterations))
                    # print "Astar v2 requests lower or equal: " + str(astOCRcount) + " with average " + str(astOCRtotal/float(iterations))
                    # print ""
                    # print "Astar runtime better: " + str(astTcount) + " with average " + str(astTtotal/float(iterations))
                    # print "Astar v2 runtime better or equal: " + str(astOCTcount) + " with average " + str(astOCTtotal/float(iterations))
                    # print ""

                # batchNormalizeAndWrite(fn2, fn1, [astOCRtotal/float(iterations), astOCTtotal/float(iterations)], [astRtotal/float(iterations), astTtotal/float(iterations)], [norm, norm])
                batchNormalizeAndWrite(fn2, fn1, [astOCCtotal/float(iterations), astOCRtotal / float(iterations), astOCTtotal / float(iterations)], [grdCtotal/float(iterations), grdRtotal / float(iterations), grdTtotal / float(iterations)], [norm, norm, norm])

                # batchNormalizeAndWrite(fn2, fn1, [astOCC, astOCTotalR, astOCT], [grdC, grdTotalRLocal, grdT], [norm, norm, norm])


random.seed(73)

# ExhaustiveComparison()
MultithreadedAStarCompare()
