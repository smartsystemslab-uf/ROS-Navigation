#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.02f; //Meters of individual squares on pattern.png
const float arucoSquareDimension = 0.066f; //Meters of aruco square
const Size chessboardDimensions = Size(6, 9);

void createArucoMarkers() {
	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	for (int i = 0; i < 50; i++) {
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}
}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
		}
	}
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) {
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++) {
		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			allFoundCorners.push_back(pointBuf);
		}

		if (showResults) {
			drawChessboardCorners(*iter, Size(9, 6), pointBuf, found);
			imshow("Looking for Corners", *iter);
			waitKey(0);
		}
	}
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients) {
	vector<vector<Point2f>> checkerBoardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerBoardImageSpacePoints, false);

	vector<vector<Point3f>> worldSpaceCornerPoints(1);

	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerBoardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	vector<Mat> rVectors, tVectors;
	distanceCoefficients = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerBoardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) {
	ofstream outStream(name);
	if (outStream) {
		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		outStream.close();
		return true;
	}
	return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients) {
	ifstream inStream(name);
	if (inStream) {
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << "\n";
			}
		}
		//Distance Coefficients
		inStream >> rows;
		inStream >> columns;

		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {
				double read = 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << "\n";
			}
		}

		inStream.close();
		return true;

	}
	return false;
}

int getLesser(int x, int y){
	if(x<=y){
		return x;
	}else{
		return y;
	}
}

Point2f getLocationMarker(vector<Point2f> corners){
	float corner0x = corners[0].x;
	float corner0y = corners[0].y;
	float corner2x = corners[2].x;
	float corner2y = corners[2].y;
	float xDiffHalf = (abs(corner0x - corner2x))/2;
	float yDiffHalf = (abs(corner0y - corner2y))/2;
	float lesserNumberx = getLesser(corner0x, corner2x);
	float lesserNumbery = getLesser(corner0y, corner2y);
	return Point2f((lesserNumberx + xDiffHalf), (lesserNumbery + yDiffHalf));

}

float getMarkerRotationDegrees(vector<Point2f> corners){
	float corner0x = corners[0].x;
	float corner0y = corners[0].y;
	float corner1x = corners[1].x;
	float corner1y = corners[1].y;
	float xDiff = abs(corner0x - corner1x);
	float yDiff = abs(corner0y - corner1y);

	if(corner0x<corner1x){
		if(corner0y==corner1y){
			return 0.0;
		}else{
			if(corner0y<corner1y){
				return atan(yDiff/xDiff)*180/PI;
			}else{
				return -(atan(yDiff/xDiff)*180/PI);
			}
		}
	}else if(corner0x>corner1x){
		if(corner0y==corner1y){
			return 180.0;
		}else{
			if(corner0y<corner1y){
				return 180-(atan(yDiff/xDiff)*180/PI);
			}else{
				return -(180-(atan(yDiff/xDiff)*180/PI));
			}
		}
	}else{
		if(corner0y==corner1y){
			return -333.0;
		}else{
			if(corner0y>corner1y){
				return -90.0;
			}else{
				return 90.0;
			}
		}
	}
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension) {
	Mat frame;

	int framesPerSecond = 2;

	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	aruco::DetectorParameters parameters;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	/*VideoCapture vid(0);

	if (!vid.isOpened()) {
		return -1;
	}*/

	//namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	vector<Vec3d> rotationVectors, translationVectors;

	while (true) {
		VideoCapture vid(0);
		if (!vid.isOpened()) {
			return -1;
		}
		if (!vid.read(frame)) {
			break;
		}
		vid.release();

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
		//aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

		/*for (int i = 0; i < markerIds.size(); i++) {
			aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
		}

		imshow("Webcam", frame);*/
		for(int i = 0; i < markerIds.size(); i++){
			//[0] is rotation
			//cout << markerIds[i] << ": " << rotationVectors[i][0] << " " << rotationVectors[i][1] << " " << rotationVectors[i][2] << endl;
			//cout << markerIds[i] << ": " << markerCorners[i][0] << " " << markerCorners[i][1] << " " << markerCorners[i][2] << " " << markerCorners[i][3] << "   " << rotationVectors[i][0] << endl;
			cout << markerIds[i] << "   " << getLocationMarker(markerCorners[i]) << "  Rotation: " << getMarkerRotationDegrees(markerCorners[i]) << endl;	
		}
		char character = waitKey(1000 / framesPerSecond);
		if (waitKey(30) >= 0) break;
	}
	return 1;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients) {
	Mat frame;
	Mat drawToFrame;

	vector<Mat> savedImages;

	vector<vector<Point2f>> markerCorners, rejectedCandidates;

	VideoCapture vid;

	//cout << "After vid capture" << endl;

	int framesPerSecond = 5;

	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	while (true) {
		vid.open(0);
		if (!vid.read(frame))
			break;

		vector<Vec2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH);
		frame.copyTo(drawToFrame);
		drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
		if (found){
			imshow("Webcam", drawToFrame);
			Mat temp;
			frame.copyTo(temp);
			savedImages.push_back(temp);
			cout << "Image found: " << savedImages.size() << endl;
		}else
			imshow("Webcam", frame);

		vid.release();

		char character = waitKey(1000 / framesPerSecond);

		if (savedImages.size() > 50) {
			cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
			saveCameraCalibration("CameraCalibrationOutput", cameraMatrix, distanceCoefficients);
			return;
		}

		switch (character)
		{
		case ' ':
			//saving image
			if (found) {
				Mat temp;
				frame.copyTo(temp);
				savedImages.push_back(temp);
			}
			break;
		case 13:
			//start calibration
			if (savedImages.size() > 15) {
				cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
				saveCameraCalibration("CameraCalibrationOutput", cameraMatrix, distanceCoefficients);
			}
			break;
		case 27:
			//exit
			return;
			break;
		}

	}
}



int main(int argv, char** argc)
{
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;
	//cameraCalibrationProcess(cameraMatrix, distanceCoefficients); //To make calibration file
	loadCameraCalibration("CameraCalibrationOutput", cameraMatrix, distanceCoefficients);
	startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension); ///////////////////////////////////////////////////////////////////////////////////////////////////aruco square dimensions could be wrong
	
	return 0;

}
