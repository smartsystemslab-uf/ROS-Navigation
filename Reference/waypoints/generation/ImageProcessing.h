//
// Created by andrew on 5/16/17.
//

#ifndef CEILINGCAM_IMAGEPROCESSING_H
#define CEILINGCAM_IMAGEPROCESSING_H

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <iostream>


using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using cv::Rect;
using cv::Point;
using cv::Scalar;
using cv::InputArray;
using cv::OutputArray; 

class ImageProcessing {
public:
    static Mat performMeanShift(const Mat &input, bool writeFiles);
    static void SegmentsColoring(Mat& img, const Scalar& colorDiff);
    static void OpenClose(int track_pos, void*, InputArray src, OutputArray dst);
};


#endif //CEILINGCAM_IMAGEPROCESSING_H
