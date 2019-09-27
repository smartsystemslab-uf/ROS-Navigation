// sudo modprobe bcm2835-v4l2
// C++
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <chrono>
#include <cmath>
#include <typeinfo>
#include <algorithm>
#include <random>
#include <cstring>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

// OpenMP
#include <omp.h>

// own headers
#include "RobotControl/Planner.h"
#include "DataStructures/PixelState.h"
#include "Imaging/MeanShift.h"
#include "DataStructures/TransitionRegion.h"
#include "Imaging/RobotDetection.h"
#include "DataStructures/System.h"
#include "Imaging/ImageProcessing.h"
#include "Imaging/CeilingObjectDetection.h"
//#include "DataStructures/PathLookup.h"

// sockets
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <errno.h>


vector<vector<string>> grid = {{"",  "A", "B", ""},
							   {"C", "1", "2", "D"},
							/*{"G", "3", "4", "H"},*/
							   {"",  "E", "F", ""}}; // 2d vector of camera ids
char myID = '0'; // holds this camera's name which is assigned at runtime
string srcCam = ""; // holds the id of the camera/region from which the robot came in order to assume orientation
string nextCam = "";
int myGridRow = -1; // this camera's location in grid
int myGridCol = -1; // this camera's location in grid



// take image moved to ImageProcessing, static

// check valid points moved to Planner, static

bool isTowardGoal(int nIndex, int goalLocation, Ceiling c) {

    vector<Ceiling> neighbors = c.getNeighbors();

	int goalRow = -1;
	int goalCol = -1;
	cout << "Checking to see if my neighbor, camera " << neighbors[nIndex].getCamID() << " is toward goal "
		 << to_string(goalLocation) << endl;
	for (unsigned int i = 0; i < grid.size(); i++) {
		vector<string>::iterator it = find(grid[i].begin(), grid[i].end(), to_string(goalLocation));
		if (it != grid[i].end()) {
			goalRow = i;
			goalCol = distance(grid[i].begin(), it);
		}
	}
	if (goalRow != -1 && goalCol != -1) {
		//cout << "I am at (" << myGridRow << "," << myGridCol << ") and " << endl;
		//cout << " this neighbor is at (" << neighbors[nIndex].gridRow << "," << neighbors[nIndex].gridCol << ") and " << endl;
		//cout << " goal is at (" << goalRow << "," << goalCol << ")" << endl;
		int myDistToGoal = abs(myGridRow - goalRow) + abs(myGridCol - goalCol);
		//cout << "My row dist to goal is " << abs(myGridRow - goalRow) << " and my col dist to goal is " << abs(myGridCol - goalCol) << endl;
		//cout << " My total dist to goal is " << myDistToGoal << endl;
		int neighborDistToGoal = abs(neighbors[nIndex].getGridRow() - goalRow) + abs(neighbors[nIndex].getGridCol() - goalCol);
		//cout << "Neighbor row dist to goal is " << abs(neighbors[nIndex].gridRow - goalRow) << " and my col dist to goal is " << abs(neighbors[nIndex].gridCol - goalCol) << endl;
		//cout << " Neighbor total dist to goal is " << neighborDistToGoal << endl;
		if (neighborDistToGoal < myDistToGoal) { // if the neighbor is closer to the goal, return true
			return true;
		}
	}
	return false;
}

int main(int argc, char** argv){
	Ceiling current;
}
