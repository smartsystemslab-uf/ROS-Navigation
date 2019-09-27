//
// Created by andrew on 5/16/17.
//

#ifndef CEILINGCAM_CEILINGOBJECTDETECTION_H
#define CEILINGCAM_CEILINGOBJECTDETECTION_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include "Aruco.h"

using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using cv::Rect;
using cv::Point;

class CeilingObjectDetection {
    
public:
    enum pushCheckType{MIN, MAX, REPLACE_REGARDLESS, MIN_MAX};
    static vector<Point> detectObjectCenter(const Mat &regionBinary, Mat &newImage,  vector<int> &robotSizesX, vector<int> &robotSizesY);
    static Point detectObejectWithAruco(const Mat &regionBinary, const vector<Mat> &frames,
                                        vector<int> &robotSizesX, vector<int> &robotSizesY,
                                        const int id, const Aruco &ar);
    static int countNonZeroDiv(Mat &image, int divValue);
    static vector<Point> getObjectXYTiled(const Mat &region, vector<int> &robotSizesX,vector<int> &robotSizesY, int rectSize);
    static vector<Point> getObjectXYTiledNoSize(const Mat &region, int rectSize);
    static Mat getClosestImage(const vector<Mat> &takenImages, const Mat &original);
    static int getIndexOfMaxValue(int *pixelCounts);
    static void checkAndPush(int a, int b, vector<int> &aVec, vector<int> &bVec, int replacementType, int differenceMinimum);
    static void cleanVector(vector<int> &aVec, vector<int> &bVec, int distance, int replacementType);
    static bool valuesNear(int a, int b, double percentWithin);
    static bool pixelsNear(int a, int b, int pixels);
    static bool checkRightBorderPixels(const Mat &region, int lValue);
    static bool checkLeftBorderPixels(const Mat &region, int lValue);
    static bool checkTopBorderPixels(const Mat &region, int lValue);
    static bool checkBottomBorderPixels(const Mat &region, int lValue);
    static vector<double> detectOrientation(const Mat &robotRegion, vector<Point> robotCenterPoints, const vector<Rect> &robotRegionRects);
    static void detectEdges(const Mat &robotRegion);
};


#endif //CEILINGCAM_CEILINGOBJECTDETECTION_H
