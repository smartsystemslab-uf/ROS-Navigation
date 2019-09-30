//
// Created by andrew on 3/7/17.
//

#ifndef CEILINGCAM_ROBOTDETECTION_H
#define CEILINGCAM_ROBOTDETECTION_H
 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <chrono>
#include <cmath>
#include <typeinfo>

using cv::Mat;
using cv::Point;
using std::vector;

class RobotDetection {
private:
    static int maxIndex(int pixelCounts[]);
    static void checkAndPush(int a, int b, vector<int> &aVec, vector<int> &bVec, int replacementType, int differenceMinimum);
    static void cleanVector(vector<int> &aVec, vector<int> &bVec, int distance, int replacementType);
    bool valuesNear(int a, int b, double percentWithin);
    static bool pixelsNear(int a, int b, int pixels);
public:
    enum pushCheckType{MIN, MAX, REPLACE_REGARDLESS, MIN_MAX};
    static vector<Point> detectLostRobot(/*const*/ Mat &originalImage, vector<Mat> &modifiedImages,  vector<int> &robotSizesX,vector<int> &robotSizesY);
    static int countNonZeroDiv16(Mat &image);
    static vector<Point> getRobotXY(const Mat &region, vector<int> &robotSizesX,vector<int> &robotSizesY);
    static Mat getClosestImage(const vector<Mat> &takenImages, const Mat &original, int zeroPixelCounts[3], vector<Mat> &bestQuadrants);
    static void testNoiseReduction(const Mat &original, const Mat &modified);
};


#endif //CEILINGCAM_ROBOTDETECTION_H
