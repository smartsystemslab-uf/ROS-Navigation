//
// Created by afelder on 5/31/18.
//

#ifndef DISCMAHN_ARUCO_H
#define DISCMAHN_ARUCO_H


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>

using namespace cv;
using std::vector;
using std::endl;
using std::string;
using std::cout;
using std::ofstream;
using std::ifstream;

class Aruco {
public:
    Aruco(){
        cameraMatrix = Mat::eye(3,3,CV_64F);
    };

    void calibrate();
    bool loadCameraCalibration(string name);
    void testDetectByID(const string &filename, int id);
    vector<Point2f> detectByID(const vector<Mat> &frames, int id, int &index, int &frameIndex) const;

    void setCalibrationSquareDimension(float calibrationSquareDimension);
    void setArucoSquareDimension(float arucoSquareDimension);
    void setChessboardDimensions(const Size &chessboardDimensions);

    float getCalibrationSquareDimension() const;
    float getArucoSquareDimension() const;
    const Size &getChessboardDimensions() const;

private:
    void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients);
    bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients);
    void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners);
    void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false);
    Point2f getLocationMarker(vector<Point2f> corners);
    double getMarkerRotationDegrees(vector<Point2f> corners);

    Mat cameraMatrix;
    Mat distanceCoefficients;
    float calibrationSquareDimension = 0.02f; //Meters of individual squares on pattern.png
    float arucoSquareDimension = 0.066f; //Meters of aruco square
    Size chessboardDimensions = Size(6, 9);
};


#endif //DISCMAHN_ARUCO_H
