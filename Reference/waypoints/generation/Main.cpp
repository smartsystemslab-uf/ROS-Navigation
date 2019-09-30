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
#include "Planner.h"
#include "PixelState.h"
#include "MeanShift.h"
//#include "PathLookup.h"
#include "TransitionRegion.h"
#include "RobotDetection.h"
#include "System.h"
#include "ImageProcessing.h"
#include "CeilingObjectDetection.h"

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
Planner planner; // for the other algorithms
string path = ""; // global container for current path string
double cost = 9999.99; // global container for current path cost
float robotSize;

vector<int> robotSizesX; //robot detection points
vector<int> robotSizesY; //robot detection points
string robot_IP; // ip address of current robot that this camera is in communication with

Mat preprocImg, robotInView, openClose; // cv::Mat - original image, original with robot in frame, binary image

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
void takeImage() {
    VideoCapture capture(0); // camera object
    capture >> preprocImg; // Hey Joe! This is the original image.
    if (preprocImg.empty())
        return;
    //preprocImg = imread("image000.jpg");
    openClose = ImageProcessing::performMeanShift(preprocImg, true);
}

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

// returns true if the starting points and goal points are valid (not in or near obstacles)
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
string doPointPathFinding(int startXpath, int startYpath, int goalXpath, int goalYpath, int alg, Mat waypoint, double colorVal) {
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  //openClose.release(); // was getting lots of errors with this
  openClose = imread("subtractedImage.jpg");
  bufferDistance = (((sqrt(pow(robotSize, 2) + pow(robotSize, 2))) / 2) + 2) * .85;
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

  // cout << "Detected a robot of size " << robotSizesX[0] << " by " << robotSizesY[0] << endl;
  ticksPerPixel = planner.calcTicksPerPixel(ARLO_tickDistance_CM, ARLO_SIZE_CM, robotSize);

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
  // for (unsigned int i = 0; i < neighbors.size(); i++) {
  //   if (grid[neighbors[i].gridRow][neighbors[i].gridCol] == srcCam) {
  //     srcCam = to_string(neighbors[i].side);
  //     break;
  //   }
  // }
  // cout << "srcCam overwritten to neighbor.side: " << srcCam << endl;

  cout << "Attempting to find a path from (" << startXpath << "," << startYpath << ") to (";
  cout << goalXpath << "," << goalYpath << ") with a buffer of " << bufferDistance << endl;

  // check if start and goal have been calculated previously FIXME needs updating
  // if (savedPaths.size() > 0) {
  //    for (int i = 0; i < savedPaths.size(); i++) {
  //        if (savedPaths[i].start_x == startXpath && savedPaths[i].start_y == startYpath &&
  //            savedPaths[i].goal_x == goalXpath && savedPaths[i].goal_y == goalYpath) {
  //            foundSavedPath = true;
  //            p = savedPaths[i].pathString;
  //            break;
  //        }
  //    }
  // }

  // if (!foundSavedPath) { // if we cannot find a previously calculated path, run path finding
    if (alg == 0) {
      // pathResult = planner.BFS(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), path, bufferDistance, openClose, srcCam, nextCam, std::max(robotSizesX[0], robotSizesY[0]), ticksPerPixel);
    }
    else {
      pathResult = planner.AStar(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), path, bufferDistance, openClose, srcCam, nextCam, alg, robotSize, ticksPerPixel, cost, waypoint, colorVal);
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
  // }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  cout << "Path took: " << time_span.count() * 1000 << " milliseconds" << endl;

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
    // int fd_path; // filedescriptor for path
    // struct sockaddr_in client_addr; // structure containing client address
    // unsigned int clilen = sizeof(client_addr);

    PathLookup readPath; // path object for read paths
    string pathToSend = ""; // this is the path that we will send. it will be converted into byte[] using .c_str()

    unsigned int pathCount = 1; // path counter for determining when to back-up
    bool foundSavedPath = false; // bool for lookup

    vector<TransitionRegion> leftTransitions;
    vector<TransitionRegion> rightTransitions;
    vector<TransitionRegion> topTransitions;
    vector<TransitionRegion> bottomTransitions;

    // cout << "Sockets have been enabled" << endl;

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


    //pathPlanning.path_file = "path.txt";

    // cout << "Taking an image of my region and processing the image..." << endl;
    // takeImage();

    //THIS IS TEMPORARY, SHOULD BE REMOVED ONCE WE START TRYING TO ACTUALLY DETECT A ROBOT WITH THE CEILING CAMERA
//  robotInView = imread("robotInView.jpg", 0);
    preprocImg = imread("source.png");
    openClose = ImageProcessing::performMeanShift(preprocImg, true);

    Mat subtractedImage = imread("subtractedImage.jpg", 0);
    Mat waypoint = imread("subtractedImage.jpg");
    imwrite("waypointsz.jpg", waypoint);

    TransitionRegion t;
    leftTransitions = t.getLeftTransitionRegions(openClose);
    rightTransitions = t.getRightTransitionRegions(openClose);
    topTransitions = t.getTopTransitionRegions(openClose);
    bottomTransitions = t.getBottomTransitionRegions(openClose);

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

    // cout << "Which camera am I? [1,2]" << endl;
    // cin >> myID;
    int alg = 2; // 0 = BFS, 1 = AStar, 2 = AStar that PUNISHES TURNS
    start = std::chrono::high_resolution_clock::now();

    robotSize = ARLO_SIZE_CM;
    tickDistance = ARLO_tickDistance_CM;
    robot_IP = ARLO_IP;
    cout << "Robot size set to size of Arlo: " << robotSize << "cm" << endl;
    int robX, robY, goalX, goalY;
    //cout << "IF YOU WANT TO USE OTHER ROBOTS FOR TESTING, AN IMAGE WITH THOSE ROBOTS IN A TRANSITION REGION WILL BE NEEDED" << endl;

    for(int i = 0; i < leftTransitions.size();i++){
      robX = leftTransitions[i].xPoint;
      robY = leftTransitions[i].yPoint;
      for(int i = 0; i < rightTransitions.size();i++){
        goalX = rightTransitions[i].xPoint;
        goalY = rightTransitions[i].yPoint;
        // cin >> robotChoice;
        // path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, (128.0 / (double)(leftTransitions.size() + rightTransitions.size() + topTransitions.size() + bottomTransitions.size())));
        path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 15.0);
    	}
    	for(int i = 0; i < topTransitions.size();i++){
        goalX = topTransitions[i].xPoint;
        goalY = topTransitions[i].yPoint;
        // cin >> robotChoice;
        path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 15.0);
    	}
    	for(int i = 0; i < bottomTransitions.size();i++){
        goalX = bottomTransitions[i].xPoint;
        goalY = bottomTransitions[i].yPoint;
        // cin >> robotChoice;
        path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 15.0);
      }
  	}
  	for(int i = 0; i < rightTransitions.size();i++){
      robX = rightTransitions[i].xPoint;
      robY = rightTransitions[i].yPoint;
      // for(int i = 0; i < leftTransitions.size();i++){
      //   goalX = leftTransitions[i].xPoint;
      //   goalY = leftTransitions[i].yPoint;
      //   // cin >> robotChoice;
      //   path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 5.0);
    	// }
    	for(int i = 0; i < topTransitions.size();i++){
        goalX = topTransitions[i].xPoint;
        goalY = topTransitions[i].yPoint;
        // cin >> robotChoice;
        path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 15.0);
    	}
    	for(int i = 0; i < bottomTransitions.size();i++){
        goalX = bottomTransitions[i].xPoint;
        goalY = bottomTransitions[i].yPoint;
        // cin >> robotChoice;
        path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 15.0);
      }
  	}
  	for(int i = 0; i < topTransitions.size();i++){
      robX = topTransitions[i].xPoint;
      robY = topTransitions[i].yPoint;
      // for(int i = 0; i < leftTransitions.size();i++){
      //   goalX = leftTransitions[i].xPoint;
      //   goalY = leftTransitions[i].yPoint;
      //   // cin >> robotChoice;
      //   path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 5.0);
    	// }
    	// for(int i = 0; i < rightTransitions.size();i++){
      //   goalX = rightTransitions[i].xPoint;
      //   goalY = rightTransitions[i].yPoint;
      //   // cin >> robotChoice;
      //   path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 5.0);
    	// }
    	for(int i = 0; i < bottomTransitions.size();i++){
        goalX = bottomTransitions[i].xPoint;
        goalY = bottomTransitions[i].yPoint;
        // cin >> robotChoice;
        path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 15.0);
      }
  	}
  	// for(int i = 0; i < bottomTransitions.size();i++){
    //   robX = bottomTransitions[i].xPoint;
    //   robY = bottomTransitions[i].yPoint;
    //   for(int i = 0; i < leftTransitions.size();i++){
    //     goalX = leftTransitions[i].xPoint;
    //     goalY = leftTransitions[i].yPoint;
    //     // cin >> robotChoice;
    //     path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 5.0);
    // 	}
    // 	for(int i = 0; i < topTransitions.size();i++){
    //     goalX = topTransitions[i].xPoint;
    //     goalY = topTransitions[i].yPoint;
    //     // cin >> robotChoice;
    //     path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 5.0);
    // 	}
    // 	for(int i = 0; i < rightTransitions.size();i++){
    //     goalX = rightTransitions[i].xPoint;
    //     goalY = rightTransitions[i].yPoint;
    //     // cin >> robotChoice;
    //     path = doPointPathFinding(robX, robY, goalX, goalY, alg, waypoint, 5.0);
    //   }
    // }

    end = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    cout << "Waypoints took total: " << time_span.count() * 1000 << " milliseconds" << endl;
    cout << "Waypoints found a total of " << Planner::totalPixels << " (potentially non-unique) pixels" << endl;
    Planner::hash.Print();
    vector<int> wayp = Planner::hash.getPointsOfX(5);
    Mat testHT = imread("subtractedImage.jpg");
    for (size_t i = 0; i < wayp.size(); i++) {
      cv::rectangle(testHT, Point(wayp[i] % 1000 - 2, wayp[i] / 1000 - 2), Point(wayp[i] % 1000 + 2, wayp[i] / 1000 + 2), Vec3b(10,10,255), CV_FILLED, LINE_8, 0);
    }
    imwrite("testHTWayp.jpg", testHT);

    cout << "Waypoint size: " << wayp.size() << endl;
    for (size_t i = 0; i < wayp.size(); i++) {
      for (size_t j = 0; j < wayp.size(); j++) {
        if (wayp[i] == wayp[j])
          continue;
        // cout << "Testing i,j " << i << "," << j << " line between (" << wayp[i] % 1000 << "," << wayp[i] / 1000 << ") and (" << wayp[j] % 1000 << "," << wayp[j] / 1000 << ")." << endl;
        cv::LineIterator it(testHT, Point(wayp[i] % 1000, wayp[i] / 1000), Point(wayp[j] % 1000, wayp[j] / 1000));
        bool lineDraw = true;
        int neighborhood = 15;
        for (size_t iter = 0; iter < it.count; iter++, ++it) {
          if (!lineDraw)
            break;
          int rowStart  = max(it.pos().y - neighborhood, 0);
          int rowFinish = min(it.pos().y + neighborhood, 479);
          int colStart  = max(it.pos().x - neighborhood, 0);
          int colFinish = min(it.pos().x + neighborhood, 639);
          for (size_t curRow = rowStart; curRow <= rowFinish; curRow++) {
            if (!lineDraw)
              break;
            for (size_t curCol = colStart; curCol <= colFinish; curCol++) {
              Vec3b tmpColor = testHT.at<Vec3b>(Point(curCol, curRow));
              if ((tmpColor[0] < 5 && tmpColor[1] < 5 && tmpColor[2] < 5)) {// && !(c.isAroundObstacle(c, -1, 1, 10, subtractedImage))) {
                // cout << "    found a " << testHT.at<Vec3b>(it.pos()) << " color pixel" << endl;
                lineDraw = false;
                break;
              }
            }
          }
        }
        if (lineDraw)
          cv::line(testHT, Point(wayp[i] % 1000, wayp[i] / 1000), Point(wayp[j] % 1000, wayp[j] / 1000), Vec3b(255,10,10));
      }
    }
    imwrite("testHTWaypLines.jpg", testHT);

    /* backup paths to file. */
    imwrite("waypoints.jpg", waypoint);
    cout << "Exiting. Writing paths to file" << endl;
    ofstream outfile;
    outfile.open("savedPaths.txt");
    if (outfile.fail())
        cout << "Output file failed to open. Something is wrong..." << endl;
    for (int i = 0; i < savedPaths.size(); i++) {
        outfile << savedPaths[i] << endl;
    }
    outfile.close();

    // contours testing
    Mat cannysrc2 = imread("subtractedImage.jpg");
    Mat cannysrc = imread("subtractedImage.jpg");
    cv::bilateralFilter(cannysrc2, cannysrc, 5, 5, 80);
    imwrite("bilateralFiltOut.jpg", cannysrc);
    // Mat cannysrc = imread("originalImage.jpg");
    // cv::cvtColor(cannysrc, cannysrc, COLOR_BGR2GRAY);
    Mat cannyout;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    cv::Canny(cannysrc, cannyout, 100, 200, 3);
    imwrite("canny.jpg", cannyout);
    cv::findContours(cannyout, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0,0));

    Mat drawing = Mat::zeros(cannyout.size(), CV_8UC3);
    Mat drawing2 = Mat::zeros(cannyout.size(), CV_8UC3);
    Mat drawing3 = Mat::zeros(cannyout.size(), CV_8UC3);
    Mat drawing5 = Mat::zeros(cannyout.size(), CV_8UC3);
    // Mat drawing4 = Mat::zeros(cannyout.size(), CV_8UC3);
    Mat drawing4 = imread("testHTWayp.jpg");
    Point currpt, nextpt, prevpt, extppt, extnpt;
    size_t waypoint_distance = bufferDistance * 1.15; // FIXME this is the distance from obstacle for waypoint. this needs a real value
    for (size_t i = 0; i < contours.size(); i++) {
      // vector<Point> polycontours;
      // double epsilon = 0.01 * cv::arcLength(contours[i], true);
      // cv::approxPolyDP(contours[i], polycontours, epsilon, true);
      Scalar colorScalar = Scalar(255,255,255);
      // for (size_t j = 0; j < polycontours.size(); j++) {
      //   if (j == polycontours.size() - 1)
      //     cv::line(drawing2, Point(polycontours[i].x, polycontours[i].y), Point(polycontours[0].x, polycontours[0].y), Vec3b(10,255,10));
      //   else
      //     cv::line(drawing2, Point(polycontours[i].x, polycontours[i].y), Point(polycontours[i+1].x, polycontours[i+1].y), Vec3b(10,255,10));
      // }
      // imwrite("contours2.jpg", drawing2);
      // cv::drawContours(drawing, contours, i, colorScalar, -1, 8, hierarchy, 0, Point());
      cv::approxPolyDP(Mat(contours[i]), contours[i], 4, true);
      cv::drawContours(drawing, contours, i, colorScalar, -1, 8, hierarchy, 0, Point());
      cv::drawContours(drawing2, contours, i, colorScalar, 2, 8, hierarchy, 0, Point());
      for (size_t j = 0; j < contours[i].size(); j++) {
        if (j == contours[i].size() - 1)
          cv::line(drawing3, Point(contours[i][j].x, contours[i][j].y), Point(contours[i][0].x, contours[i][0].y), Vec3b(10,255,10));
        else
          cv::line(drawing3, Point(contours[i][j].x, contours[i][j].y), Point(contours[i][j+1].x, contours[i][j+1].y), Vec3b(10,255,10));
      }
      // EXTRA WAYPOINTS HERE
      drawing5 = drawing3;
      prevpt = contours[i][0];
      for (size_t j = 1; j < contours[i].size() - 1; j++) {
        currpt = contours[i][j];
        nextpt = contours[i][j+1];

        if (prevpt.x == currpt.x) {
          extppt.x = currpt.x;
          extppt.y = (prevpt.y > currpt.y) ? (currpt.y - waypoint_distance) : (currpt.y + waypoint_distance);
        }
        else if (prevpt.y == currpt.y) {
          extppt.x = (prevpt.x > currpt.x) ? (currpt.x - waypoint_distance) : (currpt.x + waypoint_distance);
          extppt.y = currpt.y;
        }
        else {
          extppt.x = (prevpt.x > currpt.x) ? (currpt.x - waypoint_distance) : (currpt.x + waypoint_distance);
          extppt.y = (prevpt.y > currpt.y) ? (currpt.y - waypoint_distance) : (currpt.y + waypoint_distance);
        }
        if (nextpt.x == currpt.x) {
          extnpt.x = currpt.x;
          extnpt.y = (nextpt.y > currpt.y) ? (currpt.y - waypoint_distance) : (currpt.y + waypoint_distance);
        }
        else if (nextpt.y == currpt.y) {
          extnpt.x = (nextpt.x > currpt.x) ? (currpt.x - waypoint_distance) : (currpt.x + waypoint_distance);
          extnpt.y = currpt.y;
        }
        else {
          extnpt.x = (nextpt.x > currpt.x) ? (currpt.x - waypoint_distance) : (currpt.x + waypoint_distance);
          extnpt.y = (nextpt.y > currpt.y) ? (currpt.y - waypoint_distance) : (currpt.y + waypoint_distance);
        }
        Point midpoint;
        midpoint.x = (extppt.x + extnpt.x) / 2;
        midpoint.y = (extppt.y + extnpt.y) / 2;

        bool lineDraw = true;
        int neighborhood = waypoint_distance/2;
        int rowStart  = max(midpoint.y - neighborhood, 0);
        int rowFinish = min(midpoint.y + neighborhood, 479);
        int colStart  = max(midpoint.x - neighborhood, 0);
        int colFinish = min(midpoint.x + neighborhood, 639);
        for (size_t curRow = rowStart; curRow <= rowFinish; curRow++) {
          if (!lineDraw)
            break;
          for (size_t curCol = colStart; curCol <= colFinish; curCol++) {
            Vec3b tmpColor = drawing4.at<Vec3b>(Point(curCol, curRow));
            if ((tmpColor[0] < 5 && tmpColor[1] < 5 && tmpColor[2] < 5)) {// && !(c.isAroundObstacle(c, -1, 1, 10, subtractedImage))) {
              // cout << "    found a " << testHT.at<Vec3b>(it.pos()) << " color pixel" << endl;
              lineDraw = false;
              break;
            }
          }
        }
        if (lineDraw) {
          cv::rectangle(drawing4, Point(midpoint.x - 2, midpoint.y - 2), Point(midpoint.x + 2, midpoint.y + 2), Vec3b(10,255,10), CV_FILLED, LINE_8, 0);
          cv::rectangle(drawing5, Point(midpoint.x - 2, midpoint.y - 2), Point(midpoint.x + 2, midpoint.y + 2), Vec3b(255,50,50), CV_FILLED, LINE_8, 0);
        }
        prevpt = currpt;
      }
      // cout << contours[i] << endl;
      // END EXTRA WAYPOINTS
    }
    imwrite("contours_filled.jpg", drawing);
    imwrite("contours.jpg", drawing2);
    imwrite("contours3.jpg", drawing3);
    imwrite("contours_dots.jpg", drawing4);
    imwrite("contoured_dots.jpg", drawing5);

    Mat rects = Mat::zeros(cannyout.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i = hierarchy[i][0]) {
      Rect r= boundingRect(contours[i]);
      if(hierarchy[i][2]<0 && hierarchy[i][3]<0) //Check if there is a child contour
        cv::rectangle(rects,Point(r.x-1,r.y-1), Point(r.x+r.width+1,r.y+r.height+1), Scalar(0,0,255),2,8,0); //Opened contour
      else
        cv::rectangle(rects,Point(r.x-1,r.y-1), Point(r.x+r.width+1,r.y+r.height+1), Scalar(0,255,0),2,8,0); //closed contour
    }
    imwrite("contour_attempt.jpg", rects);


    // Mat cannyout_buf;//(cannyout.size(), CV_8UC3);
    // // cv::copyMakeBorder(cannyout, cannyout_buf, 2, 2, 2, 2, BORDER_REPLICATE);
    // Scalar blackColor = Scalar(0,0,0);
    // cv::copyMakeBorder(cannyout, cannyout_buf, 2, 2, 2, 2, BORDER_CONSTANT, blackColor);
    // vector<vector<Point>> contours_buf;
    // vector<Vec4i> hierarchy_buf;
    // cv::findContours(cannyout_buf, contours_buf, hierarchy_buf, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0,0));
    // Mat drawing3 = Mat::zeros(cannyout.size(), CV_8UC3);
    // for (size_t i = 0; i < contours_buf.size(); i++) {
    //   Scalar colorScalar = Scalar(255,255,255);
    //   cv::approxPolyDP(Mat(contours_buf[i]), contours_buf[i], 4, true);
    //   cv::drawContours(drawing3, contours_buf, i, colorScalar, -1, 8, hierarchy_buf, 0, Point(-2,-2));
    //   cout << contours_buf[i] << endl;
    // }
    // imwrite("contours_filled_buff.jpg", drawing3);

    destroyAllWindows();

    return 0;
}
