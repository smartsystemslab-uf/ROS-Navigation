/*
 *  File name: main.cpp
 *	Code explanation: OpenCV Algorithm for Background subtraction with the Raspberry Pi and Raspicam

 *	Source: http://docs.opencv.org/3.1.0/d1/dc5/tutorial_background_subtraction.html#gsc.tab=0
 *  Created on: Mar 24, 2016
 *  Author: streitfr
 */

//C++
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

// Raspicam
#include <raspicam_cv.h>

//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

// OpenMP
#include <omp.h>

// own headers
#include "PathPlanning.h"
#include "PixelState.h"
#include "MovementDetection.h"
#include "MeanShift.h"
#include "SocketConnection.h"

using namespace cv;
using namespace std;

// Global variables
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

//Object for meanshift filtering
MyMeanShift myfilter;
TermCriteria termcrit = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
		5, 1);
// parameters for robot lcoation
int volatile old_xCor, old_yCor, yCor, xCor = 0;

//Values for send routine
bool destination_found = false;
bool path_updated = false;
int frame_cnt = 0;

// callback function for open/close trackbar
void OpenClose(int track_pos, void*, InputArray src, OutputArray dst) {
	int n = open_close_pos - max_iters;
	int an = n > 0 ? n : -n;
	Mat element = getStructuringElement(element_shape,
			Size(an * 2 + 1, an * 2 + 1), Point(an, an));
	if (n < 0)
		morphologyEx(src, dst, MORPH_OPEN, element);
	else
		morphologyEx(src, dst, MORPH_CLOSE, element);
}

//this colors the segmentations
static void SegmentsColoring(Mat& img,
		const Scalar& colorDiff = Scalar::all(1)) {

	RNG rng = theRNG();
	Mat mask(img.rows + 2, img.cols + 2, CV_8UC1, Scalar::all(0));

	for (int row = 0; row < img.rows; row++) { // time in sec. 0.0857902
		for (int col = 0; col < img.cols; col++) {
			if (mask.at<uchar>(row + 1, col + 1) == 0) {
				Scalar newVal(rng(256), rng(256), rng(256));
				floodFill(img, mask, Point(col, row), newVal, 0, colorDiff,
						colorDiff, 4);
			}
		}
	}
}

static void SegmentsFiltering(int, void*) {

	myfilter.MeanShiftFiltering(test_mat, res, spatialRad, colorRad,
			maxPyrLevel, termcrit);

	SegmentsColoring(res, Scalar::all(2));
//    cout << "OpenClose=" << open_close_pos << "; "
//		 << "spatialRad=" << spatialRad << "; "
//         << "colorRad=" << colorRad << "; "
//         << "maxPyrLevel=" << maxPyrLevel << endl;
//	imshow(ProcessWindow, res);
	cvtColor(res, gray, CV_BGR2GRAY);

	// Calculate histogram
	calcHist(&gray, 1, 0, Mat(), hist, 1, &histSize, ranges, true, false);

	minMaxLoc(hist, 0, 0, 0, &maxVal_loc);

	// raw pointer access for better performance
	// set values to 255 for free path or 0 for obstacle
	for (size_t row = 0; row < gray.rows; row++) { // time in sec. 0.00109901
		unsigned char *ptr = gray.ptr(row);
		for (size_t col = 0; col < gray.cols; col++) {
			unsigned char * uc_pixel = ptr;
			if (uc_pixel[0] == maxVal_loc.y) {
				uc_pixel[0] = 255; // free path white
			} else {
				uc_pixel[0] = 0; // obstacle black
			}
			ptr++;
		}
	}

	// use a combination of open and close operation for better results
	OpenClose(open_close_pos, 0, gray, open_close);
	erode(open_close, open_close,
			getStructuringElement(MORPH_RECT,
					Size(2 * robot_size + 1, 2 * robot_size + 1),
					Point(robot_size, robot_size)));
	path = open_close; // copy the open/close image into a new image
	//imshow(TestWindow, open_close);

	// write image into array for path finding process
	// create an obstacle map with 1 for obstacle and 0 for free path
	Array.resize(open_close.rows);

	for (size_t row = 0; row < open_close.rows; ++row) {
		uchar *ptr = open_close.ptr(row);
		Array[row].resize(open_close.cols);
		for (size_t col = 0; col < open_close.cols; col++) {
			uchar * uc_pixel = ptr;
			if (uc_pixel[0] == 255) {
				Array[row][col] = 0; // free path white
			} else {
				Array[row][col] = 1; // obstacle black
			}
			ptr++;
		}
	}
}

Mat preprocessImages(Mat& input_img) {
	Mat output_img;
	// Convert image to gray for better for better performance
	cvtColor(input_img, output_img, CV_BGR2GRAY);
	// Use Gaussian filter to remove noise
	GaussianBlur(output_img, output_img, Size(11, 11), 3.5, 3.5);

	return output_img;
}

void createWindows() {
	namedWindow(SourceWindow, CV_WINDOW_AUTOSIZE);
	//namedWindow(ProcessWindow, CV_WINDOW_AUTOSIZE);
	//namedWindow(TestWindow, CV_WINDOW_AUTOSIZE);
	namedWindow(PathWindow, CV_WINDOW_AUTOSIZE);
	//namedWindow(AlternativPath, CV_WINDOW_AUTOSIZE);

	createTrackbar("Open/Close", TestWindow, &open_close_pos, max_iters * 2 + 1,
			SegmentsFiltering);
	createTrackbar("Robot size", TestWindow, &robot_size, 21,
			SegmentsFiltering);
	createTrackbar("spatialRad", ProcessWindow, &spatialRad, 80,
			SegmentsFiltering);
	createTrackbar("colorRad", ProcessWindow, &colorRad, 255,
			SegmentsFiltering);
	createTrackbar("maxPyrLevel", ProcessWindow, &maxPyrLevel, 5,
			SegmentsFiltering);
}

Mat drawIntoImage(Mat& input_img) {

	// draw vertical line in the middle of the picture
	line(input_img, Point(input_img.cols / 2, 0),
			Point(input_img.cols / 2, input_img.rows), CV_RGB(0, 255, 0));
	// draw horizontal line in the middle of the picture
	line(input_img, Point(0, input_img.rows / 2),
			Point(input_img.cols, input_img.rows / 2), CV_RGB(0, 255, 0));

	/// Draw polygonal contour + bonding circles
	// Approximate contours to polygons + get bounding circles
	vector<vector<cv::Point> > contours_poly(contours_dect.size());
	vector<cv::Point2f> center(contours_dect.size());
	vector<float> radius(contours_dect.size());

	for (int i = 0; i < contours_dect.size(); i++) {
		cv::approxPolyDP(Mat(contours_dect[i]), contours_poly[i], 1, true);
		cv::minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);
	}

	for (int i = 0; i < contours_dect.size(); i++) {
		//int get_radius_size = radius[i]; for debugging
		if (radius[i] > 20) { // radius size give of the detected contour
			xCor = center[i].x; //to_string(center[i].x - (input_img.cols / 2)); // for printing location on the display
			yCor = center[i].y; //to_string((input_img.rows / 2) - center[i].y);
			cv::circle(input_img, center[i], 6.0, CV_RGB(0, 255, 0), 5, 8, 0);
			cv::line(input_img,
					cv::Point(input_img.cols / 2, input_img.rows / 2),
					center[i], CV_RGB(0, 255, 0));
		}
	}
	// then put the text itself
	putText(input_img, "X-Coordinate: " + xCor, Point(5, 15),	//
	FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 255, 0));
	putText(input_img, "Y-Coordinate: " + yCor, Point(5, 35),	//
	FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 255, 0));

	return input_img;
}
/*
 *  function to set the parameters of the pi camera
 */
float getParamVal(string param, int argc, char **argv, float defvalue = -1) {
	int idx = -1;
	for (int i = 0; i < argc && idx == -1; i++)
		if (string(argv[i]) == param)
			idx = i;
	if (idx == -1)
		return defvalue;
	else
		return atof(argv[idx + 1]);
}

int main(int argc, char **argv) {

	raspicam::RaspiCam_Cv cam; // Raspicam object
	cam.set(CV_CAP_PROP_FRAME_WIDTH, getParamVal("-w", argc, argv, 320));
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, getParamVal("-h", argc, argv, 240));
	cam.set(CV_CAP_PROP_FPS, getParamVal("-fps", argc, argv, 90));
	// setup Raspicam, color video
	if (!cam.open())
		return 1;

	PathPlanning path_planning; // for the Wavefront algorithm
	Planner<PixelState> planner; // for the other algorithms
	State<PixelState> pathResult; // to get the costs of the other algorithms
	MovementDetection detector;
	SocketConnection connection;
	omp_set_nested(1);

	/***
	 *  Create windows and trackbars
	 ***/
	createWindows();

	path_planning.path_file = "/home/pi/Desktop/path.txt";
	bool Server_con = connection.setupServer();
	bool send = false;

#pragma omp parallel num_threads(3) //explicitly specify the number of threads to be created in the team
	{

		for (;;) {

			/***
			 *  Use raspicam to capture continues images
			 ***/
			//double f = (double) getTickCount(); // function to measure time
			cam.grab();

			switch (omp_get_thread_num()) {
			case 0: {

				/***
				 *  Image preprocessing
				 ***/
				cam.retrieve(Pi_img);
				preproc_img = preprocessImages(Pi_img);

				/***
				 *  Background Foreground subtraction
				 ***/
				detector.MovementDetector(preproc_img);

				/***
				 *  Write and draw into Image
				 ***/
				contours_dect = detector.contours;
				Pi_img = drawIntoImage(Pi_img);
				imshow(SourceWindow, Pi_img);

			}
				break;

			case 1: {

//				test_mat = imread("/home/pi/SplitImgPaper_org.png", //  room, test.jpg
//						CV_LOAD_IMAGE_COLOR);
				cam.retrieve(img_org);
				frame_cnt++;
//				resize the picture for less data traffic
				//resize(img_org, img_org, Size(320, 240));
				// switch to other color space to reduce processing power
				cvtColor(img_org, test_mat, CV_BGR2Lab);

				SegmentsFiltering(0, 0);

				path_planning.useMap(Array);

				//cout << "Robot Startpoint x " << xCor << " y " << yCor << endl;
				if ((xCor != old_xCor) || (yCor != old_yCor)) {
					path_planning.setRobot(xCor,yCor);
					path_planning.setGoal(20,170);
//					use wavefront algorithm to find path from the robot to the goal
					vector<vector<ushort> > map = path_planning.wavefront();
					path_updated = true;

				// else if (xCor == 20 || yCor == open_close.size().height / 2) {
				//	cout << "Robot reached destination" << endl;
				//	destination_found = true;
				//}

				old_xCor = xCor;
				old_yCor = yCor;
//
//					pathResult = planner.AStar(
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

//				cout << "costs=" << pathResult.m_cost << endl;
				//back transform for binary image
			for (size_t row = 0; row < map.size(); row++) {
				for (size_t col = 0; col < map[row].size(); col++) {
					if (map[row][col] == 1) {
						open_close.at<uchar>(row, col) = 0;
						//Pi_img.at<Vec3b>(row, col)[2] = 255; //red color
					} else {
						open_close.at<uchar>(row, col) = 255;
						}
					}
				}
				//	 Start Point --> Robot small cycle
				circle(open_close,
						Point(yCor,xCor), 4,
						CV_RGB(255, 0, 0), 3, 8);
				//	End Point --> Goal big cycle
				circle(open_close, Point(20,170), 8,
						CV_RGB(255, 0, 0), 3, 8);

				}

			}
				break;
			case 2: {
				if (frame_cnt > 7) // wait 5 frames setup time
					if ((Server_con && path_planning.foundRobot) && !send && path_updated) {
						send = connection.sendPath(path_planning.path_file);
						path_updated = false;

						imshow(PathWindow, open_close);
						//tm = ((double) getTickCount() - tm) / getTickFrequency();
						cout << "Path send "<< endl;
				}
			}
				break;
			default:
				cout << "Thread number is to high !!!" << endl;
			}
			//f = ((double) getTickCount() - f) / getTickFrequency();

			//cout << "total time: " << f << endl;
			// push the x key to close the program
			keyboard = waitKey(frameDelay);
		}
	}
	//delete capture object
	cam.release();
	destroyAllWindows();

}
