//
// Created by andrew on 5/16/17.
//

#include "ImageProcessing.h"
#include "MeanShift.h"

using cv::cvtColor;
using cv::imwrite;
using cv::TermCriteria;
using cv::MORPH_RECT;
using cv::MORPH_OPEN;
using cv::MORPH_CLOSE;
using cv::erode;
using cv::Size;
using cv::MatND;
using cv::RNG;
using cv::theRNG;
using cv::waitKey;
using cv::imshow;

/**
 * 
 * @param input -- original Image
 * @param writeFiles -- true -> write image files -- false -> do not write files
 * @return -- final image after all processing
 */
Mat ImageProcessing::performMeanShift(const Mat &input, bool writeFiles) {
    Mat originalImage, labImg, res, gray, openClose, segColor;
    MyMeanShift myfilter;
    TermCriteria termcrit = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS,
                                         5, 1);
    // Declare parameters for meanshift
    int spatialRad = 8; //8; // 0 for the room.jpg
    int colorRad = 15; //24; //  14
    int maxPyrLevel = 1; // 2
    const int elementShape = MORPH_RECT;
    const int maxIters = 10;
    int openClosePos = 14; //8
    // Initialize parameters for histogram
    int robot_size = 3;
    Point maxVal_loc;
    MatND hist;
    const int histSize = 256;    // uint8 size
    const float range[] = { 0, 255 };
    const float *ranges[] = { range };
    

    cvtColor(input, originalImage, CV_BGR2GRAY);
    cvtColor(input, labImg, CV_BGR2Lab);
    myfilter.MeanShiftFiltering(labImg, res, spatialRad, colorRad, maxPyrLevel, termcrit);
    segColor = res.clone();
    SegmentsColoring(segColor, Scalar::all(2));

    cvtColor(segColor, gray, CV_BGR2GRAY);
    //imwrite("3-cvtGray.jpg", gray);

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
    OpenClose(openClosePos, 0, gray, openClose);
    erode(openClose, openClose,
			getStructuringElement(MORPH_RECT,
			Size(2 * robot_size + 1, 2 * robot_size + 1),
			Point(robot_size, robot_size)));

    if(writeFiles){
        imwrite("0-originalImage.jpg", input);
        imwrite("originalImage.jpg", input);
        imwrite("1-labimg.jpg", labImg);
        imwrite("2-meanshift.jpg", res);
        imwrite("3-segcolor.jpg", segColor);
        imwrite("4-subtractedImage.jpg", openClose); // Hey Joe! This is the subtracted image!
        imwrite("subtractedImage.jpg", openClose);
    }
    return openClose;
}

//this colors the segmentations
void ImageProcessing::SegmentsColoring(Mat& img, const Scalar& colorDiff = Scalar::all(1)) {
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

void ImageProcessing::OpenClose(int track_pos, void*, InputArray src, OutputArray dst) {
    const int elementShape = MORPH_RECT;
    const int maxIters = 10;
    int openClosePos = 14; //8
    int n = openClosePos - maxIters;
    int an = n > 0 ? n : -n;
    Mat element = getStructuringElement(elementShape,
                                        Size(an * 2 + 1, an * 2 + 1), Point(an, an));
    if (n < 0)
        morphologyEx(src, dst, MORPH_OPEN, element);
    else
        morphologyEx(src, dst, MORPH_CLOSE, element);
}
