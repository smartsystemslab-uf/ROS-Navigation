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
//#include "PathLookup.h"
#include "DataStructures/TransitionRegion.h"
#include "Imaging/RobotDetection.h"
#include "DataStructures/System.h"
#include "Imaging/ImageProcessing.h"
#include "Imaging/CeilingObjectDetection.h"

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


using namespace cv;
using namespace std;


#define portno 60000 // port number to use
#define buf_size 4096 // max file size to receive

int rc; // holds return code of system calls
int sock; // socket desciptor
int desc; // file descriptor for socket
int pathDesc;                           // file descriptor for file to send

char const *frName = "./path.txt"; // path to file
off_t offset;                        // file offset
struct stat statBuf;               // argument to fstat


#define ACTIVITY_BOT_SIZE_CM 20.32 // was 14.4
#define ACTIVITY_BOT_tickDistance_CM 0.345
//#define ACTIVITY_BOT_IP "192.168.1.152" // top heavy cardboard activitybot
#define ACTIVITY_BOT_IP "192.168.1.130" // sleek activitybot using a shitty piece of shit pi2

#define ARLO_SIZE_CM 45.72
#define ARLO_tickDistance_CM 0.3556
#define ARLO_IP "null"

#define GOPIGO_SIZE_CM 21.5
#define GOPIGO_tickDistance_CM 1.1
#define GOPIGO_IP "192.168.1.151"


vector<vector<string>> grid = {{"",  "A", "B", ""},
                               {"C", "1", "2", "D"},
                             /*{"G", "3", "4", "H"},*/
                               {"",  "E", "F", ""}}; // 2d vector of camera ids

vector<vector<string>> gridIP = {{"",  "A",             "B",             ""},
                                 {"C", "192.168.1.146", "192.168.1.141", "D"},
                               /*{"G", "192.168.1.153", "192.168.1.154", "H"},*/
                                 {"",  "E",             "F",             ""}}; // 2d vector of camera ips
char myID = '0'; // holds this camera's name which is assigned at runtime
string srcCam = ""; // holds the id of the camera/region from which the robot came in order to assume orientation
string nextCam = "";
int myGridRow = -1; // this camera's location in grid
int myGridCol = -1; // this camera's location in grid
vector<Ceiling> neighbors; // vector of neighbors
PixelState pathResult; // to get the costs of the other algorithms
vector<PathLookup> savedPaths; // vector of completed paths // FIXME might be worth it to implement a tree for this when n cameras
//Planner planner; // for the other algorithms
string path = ""; // global container for current path string
double cost = DBL_MAX; // global container for current path cost

vector<int> robotSizesX; //robot detection points
vector<int> robotSizesY; //robot detection points
string robot_IP; // ip address of current robot that this camera is in communication with

//Mat preprocImg, robotInView, openClose; // cv::Mat - original image, original with robot in frame, binary image

const char PathWindow[] = "RobotPath"; //create GUI windows

// grid size, buffer size, robot size
//int robotSize = 54;
int bufferDistance;// = (robotSize / 2) + 1; // global that is probably not used anymore FIXME
int gridSize;// = bufferDistance/2; // global that is probably not used anymore FIXME

float x_cm_visible = 0; //area that the camera can see
float y_cm_visible = 0; //area that the camera can see
float tickDistance = 0; // distance that one tick is worth in cm for given robot
double ticksPerPixel; // how many ticks the robot needs to perform before changing a single pixel in this camera's region


// creates pop-up windows with names
void createWindows() {
    namedWindow(PathWindow, CV_WINDOW_AUTOSIZE);
    //namedWindow(SourceWindow, CV_WINDOW_AUTOSIZE);
    //namedWindow(ProcessWindow, CV_WINDOW_AUTOSIZE);
    //namedWindow(TestWindow, CV_WINDOW_AUTOSIZE);
    //namedWindow(AlternativPath, CV_WINDOW_AUTOSIZE);
}

// open video, take frame, perform segmentation on frame, return binary img
//void takeImage() {
//    VideoCapture capture(0); // camera object
//    capture >> preprocImg; // Hey Joe! This is the original image.
//    if (preprocImg.empty()){
//        cerr << "Failed to take image" << endl;
//        exit(-1);
//    }
////        return;
//    //preprocImg = imread("image000.jpg");
//    openClose = ImageProcessing::performMeanShift(preprocImg, true);
//}

// checks the neighbor at index to see if its toward to the passed goal
bool isTowardGoal(int nIndex, int goalLocation) {
    int goalRow = -1;
    int goalCol = -1;
    cout << "Checking to see if my neighbor, camera " << neighbors[nIndex].camID << " is toward goal "
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
		int neighborDistToGoal = abs(neighbors[nIndex].gridRow - goalRow) + abs(neighbors[nIndex].gridCol - goalCol);
		//cout << "Neighbor row dist to goal is " << abs(neighbors[nIndex].gridRow - goalRow) << " and my col dist to goal is " << abs(neighbors[nIndex].gridCol - goalCol) << endl;
		//cout << " Neighbor total dist to goal is " << neighborDistToGoal << endl;
		if (neighborDistToGoal < myDistToGoal) { // if the neighbor is closer to the goal, return true
			return true;
		}
	}
    return false;
}

// path finding
string doPathFinding(int startXpath, int startYpath, int goalXpath, int goalYpath) {
    // FIXME this function is currently out of commission. use doPointPathFinding FIXME
    //set our goal state
    /*PixelState::goal_x = goalXpath;
    PixelState::goal_y = goalYpath;

    cout << "size of robotSizesX: " << robotSizesX.size() << " and robotSizesY: " << robotSizesY.size() << endl;
    cout << "Detected a robot of size " << robotSizesX[0] << " by " << robotSizesY[0] << endl;
    string p = "";
    bool foundSavedPath = false;
    bufferDistance = ((sqrt(pow(robotSizesX[0], 2) + pow(robotSizesY[0], 2))) / 2) + 2;
    gridSize = bufferDistance;// * 0.95;
    //float cm_visible = ACTIVITY_BOT_SIZE_CM * (640 / 60);
    float cm_per_pixel = ACTIVITY_BOT_SIZE_CM / 60;
    ticksPerPixel = cm_per_pixel / tickDistance;
    int ticks_per_action = 41; //(int)round(gridSize*ticksPerPixel) - 10;
    cout << "Calculated ticks_per_action of " << ticks_per_action << endl;
    cout << "Original ticks per action would've been " << ((int) round(gridSize * ticksPerPixel) - 10) << endl;

    cout << "Calculated ticks_per_action of " << ticks_per_action << endl;

    openClose.release();
    openClose = imread("subtractedImage.jpg");

    cout << "Attempting to find a path from (" << startXpath << "," << startYpath << ") to (" << goalXpath << ","
         << goalYpath << ") with a grid size of " << gridSize << " and buffer of " << bufferDistance << endl;

    /* check if start and goal have been calculated previously: */
    /*if (savedPaths.size() > 0) {
        for (int i = 0; i < savedPaths.size(); i++) {
            if (savedPaths[i].start_x == startXpath && savedPaths[i].start_y == startYpath &&
                savedPaths[i].goal_x == goalXpath && savedPaths[i].goal_y == goalYpath) {
                foundSavedPath = true;
                p = savedPaths[i].pathString;
                break;
            }
        }
    }


    if (!foundSavedPath) { // if we cannot find a previously calculated path, run path finding
        int robMax = max(robotSizesX[0], robotSizesY[0]);
        pathResult = planner.BFS(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), gridSize, path,
                                 bufferDistance, ticks_per_action, robMax, openClose);
        if (pathResult == PixelState(startXpath, startYpath)) { // if no path exists
            return p;
        } else { // else, a new path was found
            PathLookup newPath(startXpath, startYpath, goalXpath, goalYpath, max(robotSizesX[0], robotSizesY[0]),
                               path); // temporary object for pushing back new paths to vector
            savedPaths.push_back(newPath);
            p = path;
            //openClose = pathResult.m_stateInfo.m_map;
            circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
            circle(openClose, Point(goalXpath, goalYpath), 8, CV_RGB(255, 0, 0), 3, 8);
            imwrite("pathOnBinary.jpg", openClose);
        }
    }*/
    return ""; // return p
}


// DONE
//// returns true if the starting points and goal points are valid (not in or near obstacles)
bool checkValidPoints(int &startXpath, int &startYpath, int &goalXpath, int &goalYpath) {
  if (startXpath < 20)
    startXpath = 20;
  if (startYpath < 20)
    startYpath = 20;
  if (goalXpath < 20)
    goalXpath = 20;
  if (goalYpath < 20)
    goalYpath = 20;
  if (startXpath > 620)
    startXpath = 620;
  if (startYpath > 460)
    startYpath = 460;
  if (goalXpath > 620)
    goalXpath = 620;
  if (goalYpath > 460)
    goalYpath = 460;
  if (openClose.at<Vec3b>(Point(startXpath, startYpath)) == Vec3b(0,0,0) || openClose.at<Vec3b>(Point(startXpath, startYpath)) == Vec3b(1,1,1)) {
    return false;
  }
  if (openClose.at<Vec3b>(Point(goalXpath, goalYpath)) == Vec3b(0,0,0) || openClose.at<Vec3b>(Point(goalXpath, goalYpath)) == Vec3b(1,1,1)) {
    return false;
  }

  for (int i = -bufferDistance/2; i <= bufferDistance/2; i++) {
    for (int j = -bufferDistance/2; j <= bufferDistance/2; j++) {
      if ((openClose.rows > startYpath + i) && (startYpath + i >= 0) && (openClose.cols > startXpath + j) && (startXpath + j >= 0)) {
        Vec3b color = openClose.at<Vec3b>(Point(startXpath + j, startYpath + i));
        if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
          return false;
        }
      }
     /* if ((openClose.rows > goalYpath + i) && (goalYpath + i >= 0) && (openClose.cols > goalXpath + j) && (goalXpath + j >= 0)) {
        Vec3b color = openClose.at<Vec3b>(Point(goalXpath + j, goalYpath + i));
        if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
          return false;
        }
      }*/
    }
  }
  for (int i = -bufferDistance; i <= bufferDistance; i++) {
    for (int j = -bufferDistance; j <= bufferDistance; j++) {
		 if ((openClose.rows > goalYpath + i) && (goalYpath + i >= 0) && (openClose.cols > goalXpath + j) && (goalXpath + j >= 0)) {
			Vec3b color = openClose.at<Vec3b>(Point(goalXpath + j, goalYpath + i));
			if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
			  return false;
			}
		      }

    }
  }

  return true;
}

// point path finding
string doPointPathFinding(int startXpath, int startYpath, int goalXpath, int goalYpath, int alg) {
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  //openClose.release(); // was getting lots of errors with this
  openClose = imread("subtractedImage.jpg");
  bufferDistance = ((sqrt(pow(robotSizesX[0], 2) + pow(robotSizesY[0], 2))) / 2) + 2;
  if (!(checkValidPoints(startXpath, startYpath, goalXpath, goalYpath))) {
    cout << "You cannot start or end on an obstacle" << endl;
    //circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
    //circle(openClose, Point(goalXpath, goalYpath), 4, CV_RGB(0, 0, 255), 3, 8);
    //rectangle(openClose, Point(startXpath-30, startYpath-30), Point(startXpath+30, startYpath+30), CV_RGB(0,255,0), 1, 8);
    //rectangle(openClose, Point(goalXpath-30, goalYpath-30), Point(goalXpath+30, goalYpath+30), CV_RGB(0,255,0), 1, 8);
    //imshow("test", openClose);
    //waitKey(0);
    return "";
  }

  PixelState::goal_x = goalXpath;
  PixelState::goal_y = goalYpath;
  string p = "";
  bool foundSavedPath = false;

  cout << "Detected a robot of size " << robotSizesX[0] << " by " << robotSizesY[0] << endl;
  ticksPerPixel = planner.calcTicksPerPixel(ACTIVITY_BOT_tickDistance_CM, ACTIVITY_BOT_SIZE_CM, std::max(robotSizesX[0], robotSizesY[0]));

  // for testing w/o communication
  /*if (startXpath < 50) {
      srcCam = grid[myGridRow][myGridCol + 1]; // 2
  } else if (startXpath > 600) {
      srcCam = grid[myGridRow][myGridCol - 1]; // 6
  } else if (startYpath < 80) {
      srcCam = grid[myGridRow + 1][myGridCol]; // 4
  } else if (startYpath > 410) {
      srcCam = grid[myGridRow - 1][myGridCol]; // 0
  } else if (startXpath < startYpath) {
      srcCam = grid[myGridRow][myGridCol + 1]; // 2
  } else srcCam = grid[myGridRow][myGridCol - 1]; // 6
  cout << "srcCam detected as " << srcCam << endl; */
  // end testing w/o communication
  // once communication is set up, remove this FIXME

  // change srcCam from the starting region letter ("E") into the neighbor side (0-3) in string form
  for (unsigned int i = 0; i < neighbors.size(); i++) {
    if (grid[neighbors[i].gridRow][neighbors[i].gridCol] == srcCam) {
      srcCam = to_string(neighbors[i].side);
      break;
    }
	  else {
  		//cout << "debug:   grid[ ] is " << grid[neighbors[i].gridRow][neighbors[i].gridCol] << "  and srcCam is " << srcCam << endl;
	  }
  }
  cout << "srcCam overwritten to neighbor.side: " << srcCam << endl;

  cout << "Attempting to find a path from (" << startXpath << "," << startYpath << ") to (";
  cout << goalXpath << "," << goalYpath << ") with a buffer of " << bufferDistance << endl;

  // check if start and goal have been calculated previously FIXME needs updating
  if (savedPaths.size() > 0) {
     for (int i = 0; i < savedPaths.size(); i++) {
         if (savedPaths[i].start_x == startXpath && savedPaths[i].start_y == startYpath &&
             savedPaths[i].goal_x == goalXpath && savedPaths[i].goal_y == goalYpath) {
             foundSavedPath = true;
             p = savedPaths[i].pathString;
             break;
         }
     }
  }

  if (!foundSavedPath) { // if we cannot find a previously calculated path, run path finding
    if (alg == 0) {
      pathResult = planner.BFS(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), path, bufferDistance, openClose, srcCam, nextCam, std::max(robotSizesX[0], robotSizesY[0]), ticksPerPixel);
    }
    else {
      pathResult = planner.AStar(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), path, bufferDistance, openClose, srcCam, nextCam, alg, std::max(robotSizesX[0], robotSizesY[0]), ticksPerPixel, cost);
    }

    if (pathResult == PixelState(startXpath, startYpath)) { // if no path exists
      //circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
      //circle(openClose, Point(goalXpath, goalYpath), 8, CV_RGB(255, 0, 0), 3, 8);
      /*
      openClose.at<Vec3b>(Point(startXpath, startYpath)) = Vec3b(60,255,60); // drawing startState
    	openClose.at<Vec3b>(Point(startXpath-1, startYpath)) = Vec3b(60,255,60); // drawing startState
    	openClose.at<Vec3b>(Point(startXpath+1, startYpath)) = Vec3b(60,255,60); // drawing startState
    	openClose.at<Vec3b>(Point(startXpath, startYpath-1)) = Vec3b(60,255,60); // drawing startState
    	openClose.at<Vec3b>(Point(startXpath, startYpath+1)) = Vec3b(60,255,60); // drawing startState
    	openClose.at<Vec3b>(Point(goalXpath, goalYpath)) = Vec3b(60,255,60); // drawing goalState
    	openClose.at<Vec3b>(Point(goalXpath-1, goalYpath)) = Vec3b(60,255,60); // drawing goalState
    	openClose.at<Vec3b>(Point(goalXpath+1, goalYpath)) = Vec3b(60,255,60); // drawing goalState
    	openClose.at<Vec3b>(Point(goalXpath, goalYpath-1)) = Vec3b(60,255,60); // drawing goalState
    	openClose.at<Vec3b>(Point(goalXpath, goalYpath+1)) = Vec3b(60,255,60); // drawing goalState
      imwrite("pathOnBinary.jpg", openClose);
      */
      return p;
    }
    else { // else, a new path was found
      PathLookup newPath(startXpath, startYpath, goalXpath, goalYpath, 999/*robotSize*/, path); // temporary object for pushing back new paths to vector
      savedPaths.push_back(newPath);
      p = path;
      //circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
      //circle(openClose, Point(goalXpath, goalYpath), 8, CV_RGB(255, 0, 0), 3, 8);
      //imwrite("pathOnBinary.jpg", openClose);
    }
  }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

  return p;
}


/********************************************************
 *
 * Create socket and bind if successful created
 *
 *********************************************************/
void bindSocket() {
    struct sockaddr_in serv_addr;       // structure containing server address

    /* enable keep-alive on the socket */
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    int idletime = 120; /* in seconds */
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idletime, sizeof(idletime));

    /* First call to socket() function */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        fprintf(stderr, "unable to create socket: %s\n", strerror(errno));
        exit(1);
    }

    /* fill in socket structure */
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    /* Now bind the host address using bind() call.*/
    rc = bind(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if (rc == -1) {
        fprintf(stderr, "unable to bind to socket: %s\n", strerror(errno));
        exit(1);
    }
}

bool sendPath(char const *filename) {

    /* open the file to be sent */
    pathDesc = open(filename, O_RDONLY);

    if (pathDesc == -1) {
        cout << "ERROR unable to open or find file " << filename << endl;
        return 0;
    }

    /* get the size of the file to be sent */
    fstat(pathDesc, &statBuf);

    offset = 0; // set offset to zero
    /* copy file using sendfile */
    for (size_t size_to_send = statBuf.st_size; size_to_send > 0;) {
        ssize_t send = sendfile(desc, pathDesc, &offset, statBuf.st_size);
        if (send == -1) {
            cout << "ERROR from sendfile" << endl;
            return 0;
        }
        offset += send;
        size_to_send -= send;
    }

    cout << "Done sending" << endl;
    /* close descriptor for file that was sent */
    close(pathDesc);

    return 1;
}


/* Note: FIXME: paths are currently one directional. A->B is different than B->A. Need to fix. Using Andrew's path interpretation? FIXME */

int main(int argc, char **argv) {
    int fd_path; // filedescriptor for path
    struct sockaddr_in client_addr; // structure containing client address
    unsigned int clilen = sizeof(client_addr);

    PathLookup readPath; // path object for read paths
    string pathToSend = ""; // this is the path that we will send. it will be converted into byte[] using .c_str()

    unsigned int pathCount = 1; // path counter for determining when to back-up
    bool foundSavedPath = false; // bool for lookup

    vector<TransitionRegion> leftTransitions;
    vector<TransitionRegion> rightTransitions;
    vector<TransitionRegion> topTransitions;
    vector<TransitionRegion> bottomTransitions;

    cout << "Sockets have been enabled" << endl;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    /* read saved paths from file */
    ifstream infile;
    infile.open("savedPaths.txt");
    if (infile.fail())
        cout << "Could not open recorded path file. Check the backups" << endl;
    while (infile >> readPath.start_x) {
        infile >> readPath.start_y;
        infile >> readPath.goal_x;
        infile >> readPath.goal_y;
        infile >> readPath.robotSize;
        infile >> readPath.pathString;
        savedPaths.push_back(readPath);
    }
    infile.close();

    int algorithmChoice, robotX, robotY, robotChoice;
    float robotSize;

    //pathPlanning.path_file = "path.txt";

    cout << "Taking an image of my region and processing the image..." << endl;
    ImageProcessing::takeImage(openClose);

    //THIS IS TEMPORARY, SHOULD BE REMOVED ONCE WE START TRYING TO ACTUALLY DETECT A ROBOT WITH THE CEILING CAMERA
//  robotInView = imread("robotInView.jpg", 0);
    Mat subtractedImage = imread("subtractedImage.jpg", 0);

//    TransitionRegion t;
    leftTransitions = TransitionRegion::getLeftTransitionRegions(openClose);
    rightTransitions = TransitionRegion::getRightTransitionRegions(openClose);
    topTransitions = TransitionRegion::getTopTransitionRegions(openClose);
    bottomTransitions = TransitionRegion::getBottomTransitionRegions(openClose);

    Mat transRegionMat = subtractedImage.clone();
	for(int i = 0; i < leftTransitions.size();i++){
		rectangle(transRegionMat, Point(leftTransitions[i].xMin, leftTransitions[i].yMin), Point(leftTransitions[i].xMax, leftTransitions[i].yMax), CV_RGB(255,0,0),1,8,0);
	}
	for(int i = 0; i < rightTransitions.size();i++){
		rectangle(transRegionMat, Point(rightTransitions[i].xMin, rightTransitions[i].yMin), Point(rightTransitions[i].xMax, rightTransitions[i].yMax), CV_RGB(255,0,0),1,8,0);
	}
	for(int i = 0; i < topTransitions.size();i++){
		rectangle(transRegionMat, Point(topTransitions[i].xMin, topTransitions[i].yMin), Point(topTransitions[i].xMax, topTransitions[i].yMax), CV_RGB(255,0,0),1,8,0);
	}
	for(int i = 0; i < bottomTransitions.size();i++){
		rectangle(transRegionMat, Point(bottomTransitions[i].xMin, bottomTransitions[i].yMin), Point(bottomTransitions[i].xMax, bottomTransitions[i].yMax), CV_RGB(255,0,0),1,8,0);
	}
	imwrite("transRegion.jpg", transRegionMat);

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    cout << "Overhead: " << time_span.count() * 1000 << " milliseconds" << endl;


    cout << "Which camera am I? [1,2]" << endl;
    cin >> myID;
    int alg = 2; // 0 = BFS, 1 = AStar, 2 = AStar that PUNISHES TURNS

    cout << "Initializing server" << endl;
    bindSocket();
    bool Server_con;
    cout << "Server on..." << endl;
    char buffer[1024] = {0};
    int valRead; // contains the integer returned from read()

    cout << "Initializing neighbors" << endl;
    for (unsigned int i = 0; i < grid.size(); i++) {
        vector<string>::iterator it = find(grid[i].begin(), grid[i].end(), to_string(myID - '0'));
        if (it != grid[i].end()) {
            myGridRow = i;
            myGridCol = distance(grid[i].begin(), it);
            //cout << "My ID: " << hostname << " was found in the grid" << endl;
        }
    }

    cout << "My neighbors are: " << endl;
    if (std::isdigit((grid[myGridRow + 1][myGridCol])[0])) {
        cout << "  " << grid[myGridRow + 1][myGridCol] << " located at (" << myGridRow + 1 << "," << myGridCol << ")"
             << endl;
        Ceiling c; // i am a neighbor DOWN/SOUTH
        c.camID = atoi(grid[myGridRow + 1][myGridCol].c_str());
        c.gridRow = myGridRow + 1;
        c.gridCol = myGridCol;
        c.camIP = gridIP[c.gridRow][c.gridCol];
        c.side = 2;
        neighbors.push_back(c);
    }
    else {
      Ceiling c; // i am an empty area DOWN/SOUTH
      c.camID = -1;
      c.gridRow = myGridRow + 1;
      c.gridCol = myGridCol;
      c.camIP = gridIP[c.gridRow][c.gridCol];
      c.side = 2;
      neighbors.push_back(c);
    }
    if (std::isdigit((grid[myGridRow - 1][myGridCol])[0])) {
        cout << "  " << grid[myGridRow - 1][myGridCol] << " located at (" << myGridRow - 1 << "," << myGridCol << ")"
             << endl;
        Ceiling c; // i am a neighbor UP/NORTH
        c.camID = atoi(grid[myGridRow - 1][myGridCol].c_str());
        c.gridRow = myGridRow - 1;
        c.gridCol = myGridCol;
        c.camIP = gridIP[c.gridRow][c.gridCol];
        c.side = 0;
        neighbors.push_back(c);
    }
    else {
      Ceiling c; // i am an empty area UP/NORTH
      c.camID = -1;
      c.gridRow = myGridRow - 1;
      c.gridCol = myGridCol;
      c.camIP = gridIP[c.gridRow][c.gridCol];
      c.side = 0;
      neighbors.push_back(c);
    }
    if (std::isdigit((grid[myGridRow][myGridCol + 1])[0])) {
        cout << "  " << grid[myGridRow][myGridCol + 1] << " located at (" << myGridRow << "," << myGridCol + 1 << ")"
             << endl;
        Ceiling c; // i am a neighbor RIGHT/EAST
        c.camID = atoi(grid[myGridRow][myGridCol + 1].c_str());
        c.gridRow = myGridRow;
        c.gridCol = myGridCol + 1;
        c.camIP = gridIP[c.gridRow][c.gridCol];
        c.side = 1;
        neighbors.push_back(c);
    }
    else {
      Ceiling c; // i am an empty area RIGHT/EAST
      c.camID = -1;
      c.gridRow = myGridRow;
      c.gridCol = myGridCol + 1;
      c.camIP = gridIP[c.gridRow][c.gridCol];
      c.side = 1;
      neighbors.push_back(c);
    }
    if (std::isdigit((grid[myGridRow][myGridCol - 1])[0])) {
        cout << "  " << grid[myGridRow][myGridCol - 1] << " located at (" << myGridRow << "," << myGridCol - 1 << ")"
             << endl;
        Ceiling c; // i am a neighbor LEFT/WEST
        c.camID = atoi(grid[myGridRow][myGridCol - 1].c_str());
        c.gridRow = myGridRow;
        c.gridCol = myGridCol - 1;
        c.camIP = gridIP[c.gridRow][c.gridCol];
        c.side = 3;
        neighbors.push_back(c);
    }
    else {
      Ceiling c; // i am an empty area LEFT/WEST
      c.camID = -1;
      c.gridRow = myGridRow;
      c.gridCol = myGridCol - 1;
      c.camIP = gridIP[c.gridRow][c.gridCol];
      c.side = 3;
      neighbors.push_back(c);
    }
    //cout << "I have added " << neighbors.size() << " neighbor(s): " << endl;
    for (int i = 0; i < neighbors.size(); i++) {
      if (neighbors[i].camID != -1) {
        cout << "  Neighbor " << neighbors[i].camID << " to the " << neighbors[i].side << endl;
      }
      else {
        cout << "  Empty area to the " << neighbors[i].side << endl;
      }
    }
    cout << "I know who my neighbors are and where they are!" << endl;


    robotSize = ACTIVITY_BOT_SIZE_CM;
    tickDistance = ACTIVITY_BOT_tickDistance_CM;
    robot_IP = ACTIVITY_BOT_IP;
    cout << "Robot size set to size of Activity Bot: " << ACTIVITY_BOT_SIZE_CM << "cm" << endl;
    //cout << "IF YOU WANT TO USE OTHER ROBOTS FOR TESTING, AN IMAGE WITH THOSE ROBOTS IN A TRANSITION REGION WILL BE NEEDED" << endl;

    /*
    int robot_x_px = robotSizesX[0];
    int robot_y_px = robotSizesY[0];
    double robotx2 = pow(robot_x_px,2);
    double roboty2 = pow(robot_y_px,2);
    diagonal_px = sqrt(robotx2 + roboty2);
    cout << "diagonal: " << diagonal_px << endl;
    x_cm_visible = robotSize*(640/robot_x_px);
    y_cm_visible = robotSize*(480/robot_y_px);

    float cm_per_pixel = x_cm_visible/640; // cm/pix
    ticksPerPixel = cm_per_pixel/tickDistance; // (cm/pix)/(cm/tick) = (tick/pix)

    cout << "x_vis " << x_cm_visible << " y_vis " << y_cm_visible << endl;
    cout << 640.0/480.0 << " " << x_cm_visible/y_cm_visible << endl;
    cout << "tpp: " << ticksPerPixel << endl;
    //YES I KNOW THIS SHOULDN'T BE HARDCODED BUT RIGHT NOW JOE'S FOOT IS CAUSING THE HARDCODING
    robotX = x_locations[x_locations.size()-1];
    robotY = y_locations[y_locations.size()-1];
    */

    while (true) {
        cout << "Waiting for client to request detection..." << endl;
        rc = listen(sock, 1); // listening for the client only one client is allowed
        if (rc == -1) {
            fprintf(stderr, "listen failed: %s\n", strerror(errno));
            exit(1);
        }

        desc = accept(sock, (struct sockaddr *) &client_addr, &clilen);
//		desc = 1; //This uncommented and the above commented is to ignore all this socket nonsense so I can test without needing to do all this
        if (desc == -1) {
            fprintf(stderr, "accept failed: %s\n", strerror(errno));
            exit(1);
        } else {
            Server_con = true;
            cout << "Connection accept from address " << inet_ntoa(client_addr.sin_addr) << endl;

		        valRead = read(desc, buffer, 1024); // read the message from the client
            int goalLocation, goalX, goalY; // _ugh because variable names are taken elsewhere

            // TODO make a tokenizer func
            int j = 0;
            for (unsigned int i = 0; i < 4; i++) {
                string s;
                while (buffer[j] != ',' && j < strlen(buffer)) {
                    s += buffer[j];
                    j++;
                }
                j++; // discard the ','
                switch (i) {
                    case 0:
                        goalLocation = atoi(s.c_str());
                        break;
                    case 1:
                        goalX = atoi(s.c_str());
                        break;
                    case 2:
                        goalY = atoi(s.c_str());
                        break;
            		    case 3:
            			     srcCam = s[0];
        			         break;
                 }
            }
            cout << "I received a message from client: " << endl;
            cout << "    goalLocation = " << goalLocation << endl;
            cout << "    goalX = " << goalX << endl;
            cout << "    goalY = " << goalY << endl;
	          cout << "    srcCam = " << srcCam << endl;
            cout << endl;
            string bestPath = ""; // container for best path. gets initialized every loop

            // ATTEMPT TO DETECT ROBOT @Andrew
//			vector<Mat> robotDetectionImages;
//			for(int i = 0; i < 3;i++){

            VideoCapture capture(0); // camera object
            capture >> robotInView;
            if (robotInView.empty())
                return -1;
            imwrite("robotInView.jpg", robotInView);
//            robotDetectionImages.push_back(robotInView);
//				imwrite("robotInView" + to_string(i+1) + ".jpg", robotInView);
//			}
//			robotInView = imread("image002.jpg");
//			vector<Point> detectedRobotPoints = RobotDetection::detectLostRobot(andrewsMat, robotDetectionImages, robotSizesX, robotSizesY);
            //detectedRobotPoints.clear();
		cout << "Pre robot detection" << endl;
	    robotSizesX.clear();
	    robotSizesY.clear();
	    vector<Point> detectedRobotPoints = CeilingObjectDetection::detectObjectCenter(openClose, robotInView,
                                                                                           robotSizesX, robotSizesY);
            cout << "detect robot debug statements: " << endl;
            cout << "    " << detectedRobotPoints.size() << endl;
            for (int i = 0; i < detectedRobotPoints.size(); i++) {
                cout << "     (x,y): (" << detectedRobotPoints[i].x << ", " << detectedRobotPoints[i].y << ")\n";
//				circle(robotInView, Point(detectedRobotPoints[i].x, detectedRobotPoints[i].y), 4, CV_RGB(255, 0, 0), 3, 8);
            }
//			imshow("final", robotInView);
//			waitKey(0);
//			return 0;

            bool detected = false;
            int robX, robY;
            if (detectedRobotPoints.size() >= 1) {
                robX = detectedRobotPoints[0].x;
                robY = detectedRobotPoints[0].y;
                detected = true;
            }

            const char *response;
            if (detected) { // if robot detected, attempt to path toward goal
              if (to_string(myID - '0') == to_string(goalLocation)) { // if im goal
                if (std::isdigit(srcCam[0])){
                  nextCam = srcCam;
									for (unsigned int i = 0; i < neighbors.size(); i++) {
										if (grid[neighbors[i].gridRow][neighbors[i].gridCol] == nextCam) {
										  nextCam = to_string(neighbors[i].side);
										  break;
										}
									  }
								}
              else
                nextCam = "1"; // final pivot is to my neighbor on the right arbitrarily
                path = doPointPathFinding(robX, robY, goalX, goalY, alg);
                bestPath = path;
                imwrite("pathOnBinary.jpg", openClose);
                string r = "T,-1"; // i found robot, but there is no next camera
                response = r.c_str();
                cout << "I'm goal cam, created response of: " << r << endl;
                pathCount++;
              } else {
                    int neighbor;
                    int neighborFails = 0;
                    string r;
                    for (int i = 0; i < neighbors.size(); i++) {
                        if (isTowardGoal(i, goalLocation)) {
                            cout << "Neighbor " << neighbors[i].camID << " is toward the goal." << endl;
                            bool pathExists = false;
                            switch (neighbors[i].side) {
                                case 0:
                                    if (neighbors[i].camID != -1) {
                                      cout << "Pathing to the top. We have " << topTransitions.size() << " intermediary goals:\n";
                                      for (int j = 0; j < topTransitions.size(); j++) {
                                          cout << "(" << topTransitions[j].xPoint << ", " << topTransitions[j].yPoint << ");  ";
                                      }
                                      cout << endl;
                                      nextCam = "0";
                                      double bestCost = 99999;
                                      for (int j = 0; j < topTransitions.size(); j++) {
                                  		    string tempGlobalPath = doPointPathFinding(robX, robY, topTransitions[j].xPoint, topTransitions[j].yPoint, alg);
                                          if (tempGlobalPath != "") {
                                              if (bestCost > cost) { // if the path to this transitionRegion[j], replace path with it
                                                path = tempGlobalPath;
                                                bestPath = path;
                                                bestCost = cost;
                                                neighbor = neighbors[i].camID;
                                                imwrite("pathOnBinary.jpg", openClose);
                                                r = "T," + to_string(neighbor);
                                                cout << "We found a current best path to neighbor side " << 0
                                                     << "/top from (" << robX << "," << robY << ") to (" << topTransitions[j].xPoint
                                                     << "," << topTransitions[j].yPoint << ") and populated r with " << r << endl;
                                                pathCount++;
                                                pathExists = true;
                                              }
                                          }
                                      }
                                    }
                                    break;
                                case 1:
                                    if (neighbors[i].camID != -1) {
                                      cout << "Pathing to the right. We have " << rightTransitions.size()
                                           << " intermediary goals:\n";
                                      for (int j = 0; j < rightTransitions.size(); j++) {
                                          cout << "(" << rightTransitions[j].xPoint << ", " << rightTransitions[j].yPoint
                                               << ");  ";
                                      }
                                      cout << endl;
                                      nextCam = "1";
                                      double bestCost = 99999;
                                      for (int j = 0; j < rightTransitions.size(); j++) {
                                  		    string tempGlobalPath = doPointPathFinding(robX, robY, rightTransitions[j].xPoint, rightTransitions[j].yPoint, alg);
                                          if (tempGlobalPath != "") {
                                              if (bestCost > cost) { // if the path to this transitionRegion[j], replace path with it
                                                path = tempGlobalPath;
                                                bestPath = path;
                                                bestCost = cost;
                                                neighbor = neighbors[i].camID;
                                                imwrite("pathOnBinary.jpg", openClose);
                                                r = "T," + to_string(neighbor);
                                                cout << "We found a path to neighbor side " << 1
                                                     << "/right from (" << robX << "," << robY << ") to (" << rightTransitions[j].xPoint
                                                     << "," << rightTransitions[j].yPoint << ") and populated r with " << r << endl;
                                                pathCount++;
                                                pathExists = true;
                                              }
                                          }
                                      }
                                    }
                                    break;
                                case 2:
                                    if (neighbors[i].camID != -1) {
                                      cout << "Pathing to the bottom. We have " << bottomTransitions.size()
                                           << " intermediary goals:\n";
                                      for (int j = 0; j < bottomTransitions.size(); j++) {
                                          cout << "(" << bottomTransitions[j].xPoint << ", "
                                               << bottomTransitions[j].yPoint << ");  ";
                                      }
                                      cout << endl;
                                      nextCam = "2";
                                      double bestCost = 99999;
                                      for (int j = 0; j < bottomTransitions.size(); j++) {
                                  		    string tempGlobalPath = doPointPathFinding(robX, robY, bottomTransitions[j].xPoint, bottomTransitions[j].yPoint, alg);
                                          if (tempGlobalPath != "") {
                                              if (bestCost > cost) { // if the path to this transitionRegion[j], replace path with it
                                                path = tempGlobalPath;
                                                bestPath = path;
                                                bestCost = cost;
                                                neighbor = neighbors[i].camID;
                                                imwrite("pathOnBinary.jpg", openClose);
                                                r = "T," + to_string(neighbor);
                                                cout << "We found a path to neighbor side " << 2
                                                     << "/bottom from (" << robX << "," << robY << ") to (" << bottomTransitions[j].xPoint
                                                     << "," << bottomTransitions[j].yPoint << ") and populated r with " << r << endl;
                                                pathCount++;
                                                pathExists = true;
                                              }
                                          }
                                      }
                                    }
                                    break;
                                case 3:
                                    if (neighbors[i].camID != -1) {
                                      cout << "Pathing to the left. We have " << leftTransitions.size()
                                           << " intermediary goals:\n";
                                      for (int j = 0; j < leftTransitions.size(); j++) {
                                          cout << "(" << leftTransitions[j].xPoint << ", " << leftTransitions[j].yPoint
                                               << ");  ";
                                      }
                                      cout << endl;
                                      nextCam = "3";
                                      double bestCost = 99999;
                                      for (int j = 0; j < leftTransitions.size(); j++) {
                                  		    string tempGlobalPath = doPointPathFinding(robX, robY, leftTransitions[j].xPoint, leftTransitions[j].yPoint, alg);
                                          if (tempGlobalPath != "") {
                                              if (bestCost > cost) { // if the path to this transitionRegion[j], replace path with it
                                                path = tempGlobalPath;
                                                bestPath = path;
                                                bestCost = cost;
                                                neighbor = neighbors[i].camID;
                                                imwrite("pathOnBinary.jpg", openClose);
                                                r = "T," + to_string(neighbor);
                                                cout << "We found a path to neighbor side " << 3
                                                     << "/left from (" << robX << "," << robY << ") to (" << leftTransitions[j].xPoint
                                                     << "," << leftTransitions[j].yPoint << ") and populated r with " << r << endl;
                                                pathCount++;
                                                pathExists = true;
                                              }
                                          }
                                      }
                                    }
                                    break;
                            }
                            if (!pathExists) {
                                neighborFails++;
                                if (neighborFails == neighbors.size()) {
                                    r = "F,-1";
                                }
                            } else
                                break;
                        } // if it isn't toward the goal, we just ignore it.
                        else {
                            neighborFails++;
                            if (neighborFails == neighbors.size()) {
                                r = "F,-1";
                            }
                        }
                    }
                    response = r.c_str();
                    cout << "I'm not goal cam, created response of: " << r << endl;
                }
            } else {
                string r = "F,-1";
                response = r.c_str();
                cout << "I didn't see the bot, created response of: " << r << endl;
            }
            cout << "We are sending a response to client: " << response << endl;
            send(desc, response, strlen(response), 0); // send response to client

            // attempt to read next message where the robot says it is ready for the path to be shoved down its throat
            valRead = read(desc, buffer, 1024); // read the message from the client
            if (buffer[0] == 'T') { // robot is ready for path
              cout << "Robot is ready for path... sending... " << endl;

              ofstream outfile;
              outfile.open("mostRecentPath.txt");
              for (int i = 0; i < bestPath.size(); i++) {
                  outfile << bestPath[i] << endl;
              }
              outfile.close();

            size_t pathSize = strlen(bestPath.c_str());
            char buff[4];
            sprintf(buff, "%04d", (int)pathSize); // creates a character array[4] with the size of path string prepended with 0's
            cout << "Sending path size of " << buff << endl;
            send(desc, buff, strlen(buff), 0); // sends the size of the path to the robot so it knows path length
            if (pathSize > 0) {
              cout << "Sending bestPath: " << bestPath << endl;
              cout << "  for debug, path (MAY NOT BE SENT PATH): " << path << endl;
              send(desc, bestPath.c_str(), strlen(bestPath.c_str()), 0); // send the whole path as is. robot needs to tokenize it
	          }

        		/*
        		char const *pathFile = "mostRecentPath.txt";
            bool sent = sendPath(pathFile);
            if (sent)
                cout << "Path was successfully sent to ground robot on ceiling end." << endl;
        		*/
            //send(desc, path.c_str(), strlen(path.c_str()), 0); // fixed path size
            }

        }

        /* determine if we need to back-up paths. if so, perform back-up */
        // note: when ~10 paths are generated. the 11th will likely never be backed up (if max paths is about 15). consider changing this dependant on savedPaths.size
        if (pathCount % 5 == 0) {
            cout << "Backing up paths to file" << endl;
            ofstream outfile;
            outfile.open("savedPaths.txt");
            if (outfile.fail())
                cout << "Output file failed to open. Something is wrong..." << endl;
            for (int i = 0; i < savedPaths.size(); i++) {
                outfile << savedPaths[i];
            }
            outfile.close();
        }

        /*
        cout << "Begin recording robot..." << endl;
        Mat frame;
        int n = 0;
        string videoFilename = "out.avi";
        Size frameSize(640, 480);
        `VideoWriter DmWrite(videoFilename, CV_FOURCC('M','J','P','G'), 25, frameSize,true);
        VideoCapture capture(0);

        while (true) {
            capture >> frame;
            if (frame.empty())
                break;
            n++;
            stringstream ss;
            ss << n;
            putText(frame, ss.str() + " ground", Point(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 255), 1, CV_AA);
            DmWrite << frame;
            //sleep(10);
            if (n > 500)
                break;
        }
        */
        Planner::bestCost = 99999;
        Planner::imgCount = 0;
    }

    /* backup paths to file. */
    cout << "Exiting. Writing paths to file" << endl;
    ofstream outfile;
    outfile.open("savedPaths.txt");
    if (outfile.fail())
        cout << "Output file failed to open. Something is wrong..." << endl;
    for (int i = 0; i < savedPaths.size(); i++) {
        outfile << savedPaths[i] << endl;
    }
    outfile.close();
    destroyAllWindows();

    return 0;
}
