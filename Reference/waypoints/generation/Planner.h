/*
  Author(s): Dillon VanBuskirk, Joe Fantinel, Andrew Felder
  Date: May 26, 2017
  Email: {dtvanbus}, {jfantine}, {afelder}@uark.edu
*/

#ifndef CEILINGCAM_PLANNER_H
#define CEILINGCAM_PLANNER_H

#include <vector>
#include <iostream>
#include <set>
#include <queue>
#include <chrono>

#include "State.h"
#include "PixelState.h"
#include "PathLookup.h"
#include "HashTable.h"

using namespace std;
using std::cin;
using std::cout;
using std::endl;


class Planner {
public:
    typedef typename std::multiset<PixelState >::iterator multisetIt_t;

    static int goal_x;
    static int goal_y;
    static double bestCost;
    static int imgCount;
    size_t pops;
    size_t its;
    stack<PixelState> pathStack;
    ofstream pathFile;

    static int regionCMX;
    static int regionCMY;

    static int totalPixels; // soley for purpose of counting how many pixels found in waypoints. delete later
    static HashTable hash;

    Planner() {
        pops = 0;
        its = 0;
    }

    ~Planner() {

    }

    PixelState AStar(PixelState startState, PixelState goalState, string &path, int bufferDistance, Mat &binaryImage, const string &srcCam, const string &nextCam, int alg, int robSize, double ticksPerPixel, double &costResult, Mat &waypoint, double colorVal);
    PixelState UCS(PixelState startState, PixelState goalState, string &path, int bufferDistance, Mat &binaryImage, const string &srcCam, const string &nextCam,int robSize, double ticksPerPixel);
    PixelState BFS(PixelState startState, PixelState goalState, string &path, int bufferDistance, Mat &binaryImage, const string &srcCam, const string &nextCam,int robSize, double ticksPerPixel);

    string writePathFile(PixelState startState, int gridSize, const string &srcCam, const string &nextCam, int robSize, double ticksPerPixel);
    int determineStartOrientation(int x, int y);
    void rebuildPath(stack<PixelState> &pathStack, const int gridSize, Mat &binaryImage, const string &srcCam, Mat &waypoint, double colorVal);


    double calcTicksPerPixel(float tickDistance, float robotCM, float robotPixels);
    void calcRegionArea(float robotCM, float robotPixels, Mat region);
};

#endif //CEILINGCAM_PLANNER_H
