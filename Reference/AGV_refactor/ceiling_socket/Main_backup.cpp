
//C++
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>


// own headers
#include "PathPlanning.h"
#include "DataStructures/PixelState.h"

using namespace cv;
using namespace std;

Mat Pi_img,img_org, test_mat, preproc_img, moveDec_img, end_img, gray, res, path_mat,
		path;

//create GUI windows
const char SourceWindow[] = "Original Frame", ProcessWindow[] =
		"MeanShift Frame", TestWindow[] = "GetBinaryImage", PathWindow[] =
		"RobotPath", AlternativPath[] = "A*, BFS, UCS";

vector<vector<Point> > contours_dect; //create a 2D vector for the contours of the edge detector
vector<Point> points; //create a vector for drawing the contours

char keyboard; // keyboard input x key
const int frameDelay = 1;

vector<Vec4i> hierarchy;

// Write into files if necessary for debugging
ofstream map_file;
ofstream path_file;

// Declare parameters for meanshift
int spatialRad = 8; //8; // 0 for the room.jpg
int colorRad = 15; //24; //  14
int maxPyrLevel = 1; // 2

// Declare parameters for morphological operations
Mat errod_dilate, open_close;
const int element_shape = MORPH_RECT;
const int max_iters = 10;
int open_close_pos = 14; //8
static int robot_size = 10;

// Initialize parameters for histogram
Point maxVal_loc;
MatND hist;
const int histSize = 256;    // uint8 size
const float range[] = { 0, 255 };
const float *ranges[] = { range };

// Declare array for path planning
vector<vector<ushort> > Array;

State<PixelState> pathResult;

// parameters for robot lcoation
int volatile old_xCor, old_yCor, yCor, xCor = 0;

//goal location
int volatile x_goal = 360;
int volatile y_goal = 160;
//Values for send routine
bool destination_found = false;
bool path_updated = false;
int frame_cnt = 0;

// pathResult = planner.AStar(
//							State<PixelState>(
//									PixelState((open_close.size().height / 2) + 39,(open_close.size().width / 2) + 120, path)),
//							State<PixelState>(PixelState(5, open_close.size().width / 2 -25, path)));

//					pathResult = planner.BFS(
//					State<PixelState>(
//							PixelState((open_close.size().height / 2) + 39,(open_close.size().width / 2) + 120, path)),
//					State<PixelState>(PixelState(5, open_close.size().width / 2 -25, path)));
//					pathResult = planner.UCS(
//					State<PixelState>(
//							PixelState((open_close.size().height / 2) + 39,(open_close.size().width / 2) + 120, path)),
//					State<PixelState>(PixelState(5, open_close.size().width / 2 -25, path)));


void createWindows() {
	// namedWindow(SourceWindow, CV_WINDOW_AUTOSIZE);
	//namedWindow(ProcessWindow, CV_WINDOW_AUTOSIZE);
	//namedWindow(TestWindow, CV_WINDOW_AUTOSIZE);
	namedWindow(PathWindow, CV_WINDOW_AUTOSIZE);
	//namedWindow(AlternativPath, CV_WINDOW_AUTOSIZE);
}

int main(int argc, char **argv){

  PathPlanning path_planning; // for the Wavefront algorithm
	Planner<PixelState> planner; // for the other algorithms
	State<PixelState> pathResult; // to get the costs of the other algorithms
	/***
	 *  Create windows and trackbars
	 ***/
	createWindows();
  xCor = 120;
  yCor = 400;
	path_planning.path_file = "path.txt";
	bool send = false;
  Mat  postArrayAssignMat;
  // return -1;
  open_close = imread(argv[1]);
  postArrayAssignMat = open_close.clone();
  // imshow(PathWindow, open_close);
	// SegmentsFiltering(0, 0);
  cout << open_close.rows << endl; //480
  cout << open_close.cols << endl; //640
  Array.resize(open_close.rows);
	Point3_<uchar>* RGB;
  for (size_t row = 0; row < open_close.rows; row++) {
    // uchar *ptr = open_close.ptr(row);
    Array[row].resize(open_close.cols);
		// cout << row << endl;
		bool test = false;
    for (size_t col = 0; col < open_close.cols; col++) {
			test = false;
    	RGB = open_close.ptr<Point3_<uchar> >(row, col);
      // uchar * uc_pixel = ptr;
			// cout << row << ", " << col << endl;
      if (RGB->z == 255) {
        Array[row][col] = 0; // free path white
        postArrayAssignMat.at<Vec3b>(Point(col, row)) = Vec3b(100,100,100);
				test = true;
				// outputImage.at<Vec3b>(Point(c,r)) = Vec3b(255,255,255);
      } else {
        Array[row][col] = 1; // obstacle black
        // postArrayAssignMat.at<uchar>(row, col) = 200;

        postArrayAssignMat.at<Vec3b>(Point(col, row)) = Vec3b(200,200,200);
				test = true;
      }
			if(!test){
				cout << "test" << endl;
			}
      // ptr++;
      // cout << col << endl;
    }
    // cout << row << endl;
  }
  // imshow(PathWindow, postArrayAssignMat);
	// cout << "exit" << endl;
	path_planning.useMap(Array);
//
	if ((xCor != old_xCor) || (yCor != old_yCor)) {
		path_planning.setRobot(xCor,yCor);
		path_planning.setGoal(x_goal,y_goal);
//					use wavefront algorithm to find path from the robot to the goal
		vector<vector<ushort> > map = path_planning.wavefront();
		path_updated = true;
    cout << map.size() << endl;
    cout << map[0].size() << endl;

  	old_xCor = xCor;
  	old_yCor = yCor;
  	//back transform for binary image
      for (size_t row = 0; row < map.size(); row++) {
      	for (size_t col = 0; col < map[row].size(); col++) {

      		if (map[row][col] == 1) {
      			// open_close.at<uchar>(row, col) = 0;
		        open_close.at<Vec3b>(Point(col, row)) = Vec3b(0,0,0);
      			//Pi_img.at<Vec3b>(row, col)[2] = 255; //red color
      		} else {
      			// open_close.at<uchar>(row, col) = 255;
		        open_close.at<Vec3b>(Point(col, row)) = Vec3b(255,255,255);
      			}
      		}
      	}
      	//	 Start Point --> Robot small cycle
      	circle(open_close,
      			Point(xCor,yCor), 4,
      			CV_RGB(255, 0, 0), 3, 8);
      	//	End Point --> Goal big cycle
      	circle(open_close, Point(x_goal,y_goal), 8,
      			CV_RGB(255, 0, 0), 3, 8);

  	}
  imshow(PathWindow, open_close);
	// imshow(SourceWindow, postArrayAssignMat);
  waitKey(0);
  return 0;

}
