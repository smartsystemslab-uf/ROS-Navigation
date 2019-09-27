//
// Created by andrew on 4/6/17.
//


#include "TransitionRegion.h"

TransitionRegion::TransitionRegion() {

    xPoint = 0;
    yPoint = 0;
    xMin = 0;
    yMin = 0;
    xMax = 0;
    yMax = 0;
}

TransitionRegion::TransitionRegion( int xmin, int ymin, int xmax, int ymax) {

    xMin = xmin;
    yMin = ymin;
    xMax = xmax;
    yMax = ymax;

    xPoint = ( xMin + xMax ) /2;
    yPoint = ( yMin + yMax ) /2;
}

void TransitionRegion::setPoint(int xp, int yp) {

    xPoint = xp;
    yPoint = yp;
}

void TransitionRegion::setImage(string filename){
    testBinaryImage = imread(filename, 0);
}


bool TransitionRegion::checkBorderingPixels(int x, int y, Mat img){
    int validBorderCount = 0;
    for(int i = x-1; i <= x+1;i++){
        for(int j = y-1; j <= y+1; j++){
            if((x!=i && y != j) && i > 0 && i < img.cols && j > 0 && j < img.rows){
                if(img.at<Vec3b>(Point(i,j)).val[0]/16 != 0 && img.at<Vec3b>(Point(i,j)).val[1]/16 != 0 && img.at<Vec3b>(Point(i,j)).val[2]/16 != 0){
                    validBorderCount++;
                }
            }
        }
    }
    return validBorderCount>+3;
}

vector<TransitionRegion> TransitionRegion::getTopTransitionRegions(Mat &image){
    bool addToCurrentRegion = false;
    int ZeroPixels = 1;
    int count = -1;
    vector<TransitionRegion> topBorder;

    for( int i = 0; i < 639; i += 20 ){

//            cout << " i = " << i << endl;
//            cout << " i+19 = " << i+19 << endl;

        TransitionRegion temp(i, 0 , i+19, 59);
        Rect roi(i, 0 , 20, 60);
        Mat imageRoi = image(roi).clone();
        ZeroPixels = roi.area() - countNonZero(imageRoi);

        if( ZeroPixels == 0  ){
            if( addToCurrentRegion )
                topBorder[count] = topBorder[count] + temp;
            else{
                count++;
                topBorder.push_back(temp);
                addToCurrentRegion = true;
            }
        }
        else{

            //put smaller rectangle shit right here
            // smaller rectangle stuff has been optimized away for now.

//                cout << endl << "ZeroPixels = " << ZeroPixels << endl;
//                cout << "count = " << count << endl << endl;
            if(addToCurrentRegion){
                //topBorder[count].print();
//				count++;
                addToCurrentRegion = false;
            }
        }
//		top = top + temp;

    }
    int topBorderCount = count + 1;
    return topBorder;
}

vector<TransitionRegion> TransitionRegion::getBottomTransitionRegions(Mat &image){
    int count = -1;
    int ZeroPixels = 1;
    bool addToCurrentRegion = false;
    vector<TransitionRegion> bottomBorder;


    for( int i = 0; i < 639; i += 20 ){

//            cout << " i = " << i << endl;
//            cout << " i+19 = " << i+19 << endl;

        TransitionRegion temp(i, 419 , i+19, 479);
        Rect roi(i, 419 , 20, 60);
        Mat imageRoi = image(roi).clone();
        ZeroPixels = roi.area() - countNonZero(imageRoi);

        if( ZeroPixels == 0  ){
            if( addToCurrentRegion )
                bottomBorder[count] = bottomBorder[count] + temp;
            else{
                count++;
                bottomBorder.push_back(temp);
                addToCurrentRegion = true;
            }
        }
        else{

            //put smaller rectangle shit right here
            // smaller rectangle stuff has been optimized away for now.

//                cout << endl << "ZeroPixels = " << ZeroPixels << endl;
//                cout << "count = " << count << endl << endl;
            if(addToCurrentRegion){
//                    bottomBorder[count].print();
//				count++;
                addToCurrentRegion = false;
            }
        }
//		top = top + temp;

    }
    int bottomBorderCount = count + 1;
    return bottomBorder;

}

vector<TransitionRegion> TransitionRegion::getLeftTransitionRegions(Mat &image){

    int count = -1;
    bool addToCurrentRegion = false;
    vector<TransitionRegion> leftBorder;
    int ZeroPixels = 1;


    for( int i = 0; i < 479; i += 20 ){

//            cout << " i = " << i << endl;
//            cout << " i+19 = " << i+19 << endl;

        TransitionRegion temp(0, i , 59, i+19);
        Rect roi(0, i , 60, 20);
//            cv::imshow("test", image);
////            cv::waitKey();
        Mat imageRoi = image(roi).clone();

//            cout << imageRoi.channels() << endl;
        ZeroPixels = roi.area() - countNonZero(imageRoi);
//            cout << "test" << endl;

        if(ZeroPixels == 0){
            if( addToCurrentRegion )
                leftBorder[count] = leftBorder[count] + temp;
            else{
                count++;
                leftBorder.push_back(temp);
                addToCurrentRegion = true;
            }
        }
        else{

            //put smaller rectangle shit right here
            // smaller rectangle stuff has been optimized away for now.

//                cout << endl << "ZeroPixels = " << ZeroPixels << endl;
//                cout << "count = " << count << endl << endl;
            if(addToCurrentRegion){
//                    leftBorder[count].print();
                //count++;
                addToCurrentRegion = false;
            }
        }
//		top = top + temp;

    }
    int leftBorderCount = count + 1;
    return leftBorder;

}

vector<TransitionRegion> TransitionRegion::getRightTransitionRegions(Mat &image){

    int count = -1;
    bool addToCurrentRegion = false;
    vector<TransitionRegion> rightBorder;
    int ZeroPixels = 1;

    for( int i = 0; i < 479; i += 20 ){

//            cout << " i = " << i << endl;
//            cout << " i+19 = " << i+19 << endl;

        TransitionRegion temp(579, i , 639, i+19);
        Rect roi(579, i , 60, 20);
        Mat imageRoi = image(roi).clone();
        ZeroPixels = roi.area() - countNonZero(imageRoi);

        if( ZeroPixels == 0  ){
            if( addToCurrentRegion )
                rightBorder[count] = rightBorder[count] + temp;
            else{
                count++;
                rightBorder.push_back(temp);
                addToCurrentRegion = true;
            }
        }
        else{

            //put smaller rectangle shit right here
            // smaller rectangle stuff has been optimized away for now.

//                cout << endl << "ZeroPixels = " << ZeroPixels << endl;
//                cout << "count = " << count << endl << endl;
            if(addToCurrentRegion){
//                    rightBorder[count].print();
                //count++;
                addToCurrentRegion = false;
            }
        }
//		top = top + temp;

    }
    int rightBorderCount = count + 1;
	for(int i = 0; i < rightBorderCount;i++){
//		cout << rightBorder[rightBorderCount] << endl;
		//cout << "xmin: " << rightBorder[i].xMin << endl
	//		<< "xMax: " << rightBorder[i].xMax << endl
//			<< "yMin: " << rightBorder[i].yMin << endl
//			<< "yMax: " << rightBorder[i].yMax << endl
//			<< "xPoint: " << rightBorder[i].xPoint << endl
//			<< "yPoint: " << rightBorder[i].yPoint << endl;
	}
//        cout << rightBorderCount << endl;
    return rightBorder;
}

//passing unchangeable reference to save memory
void TransitionRegion::detectRobotTop(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &topBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y){
    unsigned long topBorderCount = topBorder.size();
    Mat roiDiff;

    for( int i = 0; i < topBorderCount; i++){
        Rect roi(topBorder[i].xMin, topBorder[i].yMin  , topBorder[i].xMax - topBorder[i].xMin + 1	, topBorder[i].yMax - topBorder[i].yMin + 1);
        Mat imageRoi = clearGroundImage(roi).clone();
        Mat robotRoi = robotImage(roi).clone();

        absdiff(imageRoi, robotRoi, roiDiff);
        //convert the images back to RGB because otherwise you can't draw on them
        cvtColor(robotRoi, robotRoi, COLOR_GRAY2BGR);
        cvtColor(roiDiff, roiDiff, COLOR_GRAY2BGR);


        erode( roiDiff, roiDiff, element);
        getRobotLocationsTopBottom(x_locations, y_locations, roiDiff, topBorder[i], robot_sizes_x, robot_sizes_y);

    }
}

//passing unchangeable reference to save memory
void TransitionRegion::detectRobotBottom(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &bottomBorder,const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y){
    unsigned long bottomBorderCount = bottomBorder.size();
    Mat roiDiff;
    for( int i = 0; i < bottomBorderCount; i++){
        Rect roi(bottomBorder[i].xMin, bottomBorder[i].yMin, bottomBorder[i].xMax - bottomBorder[i].xMin + 1, bottomBorder[i].yMax - bottomBorder[i].yMin + 1);
        Mat imageRoi = clearGroundImage(roi).clone();
        Mat robotRoi = robotImage(roi).clone();
        absdiff(imageRoi, robotRoi, roiDiff);
        erode( roiDiff, roiDiff, element);
        getRobotLocationsTopBottom(x_locations, y_locations, roiDiff, bottomBorder[i], robot_sizes_x, robot_sizes_y);
    }
}
//passing unchangeable reference to save memory
void TransitionRegion::detectRobotLeft(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &leftBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y){
    Mat roiDiff;
    
    unsigned long leftBorderCount = leftBorder.size();
    for( int i = 0; i < leftBorderCount; i++){
        Rect roi(leftBorder[i].xMin, leftBorder[i].yMin  , leftBorder[i].xMax - leftBorder[i].xMin + 1	, leftBorder[i].yMax - leftBorder[i].yMin + 1);
        Mat imageRoi = clearGroundImage(roi).clone();
        Mat robotRoi = robotImage(roi).clone();

        absdiff(imageRoi, robotRoi, roiDiff);
        //convert the images back to RGB because otherwise you can't draw on them
        cvtColor(robotRoi, robotRoi, COLOR_GRAY2BGR);
        cvtColor(roiDiff, roiDiff, COLOR_GRAY2BGR);

        erode( roiDiff, roiDiff, element);
        
        getRobotLocationsLeftRight(x_locations, y_locations, roiDiff, leftBorder[i], robot_sizes_x, robot_sizes_y);
        
    }
}

//passing unchangeable reference to save memory
void TransitionRegion::detectRobotRight(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &rightBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y){
    unsigned long rightBorderCount = rightBorder.size();
    Mat roiDiff;
    for( int i = 0; i < rightBorderCount; i++){
        Rect roi(rightBorder[i].xMin, rightBorder[i].yMin  , rightBorder[i].xMax - rightBorder[i].xMin + 1	, rightBorder[i].yMax - rightBorder[i].yMin + 1);
        Mat imageRoi = clearGroundImage(roi).clone();
        Mat robotRoi = robotImage(roi).clone();
        absdiff(imageRoi, robotRoi, roiDiff);
        erode( roiDiff, roiDiff, element);
        getRobotLocationsLeftRight(x_locations, y_locations, roiDiff, rightBorder[i], robot_sizes_x, robot_sizes_y);
    }

}
 

void TransitionRegion::print(){

    cout << endl << "xPoint = " << xPoint << endl;
    cout << "yPoint = " << yPoint << endl;
    cout << "xMin = " << xMin << endl;
    cout << "xMax = " << xMax << endl;
    cout << "yMin = " << yMin << endl;
    cout << "yMax = " << yMax << endl << endl;
}

TransitionRegion& TransitionRegion::operator=(const TransitionRegion& other) {

    xPoint = other.xPoint;
    yPoint = other.yPoint;
    xMin = other.xMin;
    yMin = other.yMin;
    xMax = other.xMax;
    yMax = other.yMax;
    return *this;
}




const TransitionRegion& TransitionRegion::operator+(const TransitionRegion& other) {
//		return (m_stateInfo == other.m_stateInfo);

    //TransistionRegion result;
    if(xMin < other.xMin)
        this->xMin = xMin;
    else
        this->xMin = other.xMin;
    if(xMax > other.xMax)
        this->xMax = xMax;
    else
        this->xMax = other.xMax;
    if(yMin < other.yMin)
        this->yMin = yMin;
    else
        this->yMin = other.yMin;
    if(yMax > other.yMax)
        this->yMax = yMax;
    else
        this->yMax = other.yMax;

    this->xPoint = (this->xMin + this->xMax) / 2;
    this->yPoint = (this->yMin + this->yMax) / 2;

    return *this;
}


void TransitionRegion::getRobotLocationsLeftRight(vector<int> &x_locations, vector<int> &y_locations, const Mat &roiDiff, const TransitionRegion border, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y) {
    //used for detecting center point of robot
    int x_min = roiDiff.cols  + 1;
    int x_max = 0;
    int y_min = roiDiff.rows + 1;
    int y_max = 0;
    int robotPixelCount = 0;
    bool whiteDetectedInRow = false;
    bool whitePreviouslyFound = false;
    int spaceAfterWhiteCount = 0;
    for(int j = 0; j < roiDiff.rows - 1; j++){
        for(int k = 0; k < roiDiff.cols - 1; k++){
            whiteDetectedInRow = false;
            if(roiDiff.at<Vec3b>(Point(k,j)).val[0]/16 != 0 && roiDiff.at<Vec3b>(Point(k,j)).val[1]/16 != 0 && roiDiff.at<Vec3b>(Point(k,j)).val[2]/16 != 0){
                if(checkBorderingPixels(k, j, roiDiff)){
                    x_min = min(x_min, k);
                    y_min = min(y_min, j);
                    x_max = max(x_max, k);
                    y_max = max(y_max, j);
//                    robotRoi.at<Vec3b>(Point(k,j)) = Vec3b(66, 134, 244);
                    robotPixelCount++;
                    spaceAfterWhiteCount = 0;
                    whiteDetectedInRow = true;
                    whitePreviouslyFound = true;
//                        cout << "white" << endl;
                }
            }
        }
//            cout << "Row " << j << " Detected: " << whiteDetectedInRow << " Previous: " << whitePreviouslyFound << endl;
        //If we've found a robot at some point and have 10 rows without a white pixel detected, push shit
        if(whitePreviouslyFound && !whiteDetectedInRow){
            whiteDetectedInRow = false;
            spaceAfterWhiteCount++;
//                cout << spaceAfterWhiteCount << endl;
            if(spaceAfterWhiteCount == 10){
                whitePreviouslyFound = false;
                spaceAfterWhiteCount = 0;
                cout << "x_min: " << x_min << " x_max: " << x_max << endl;
                cout << "y_min: " << y_min << " y_max: " << y_max << endl;

                int x_center = (x_min + x_max)/2;
                int y_center = (y_min + y_max)/2;

                x_locations.push_back(border.xMin + x_center);
                y_locations.push_back(border.yMin + y_center);
                robot_sizes_x.push_back(x_max - x_min);
                robot_sizes_y.push_back(y_max - y_min);

                x_min = roiDiff.cols + 1;
                x_max = 0;
                y_max = 0;
                y_min = roiDiff.rows + 1;
            }
        }
    }
}


void TransitionRegion::getRobotLocationsTopBottom(vector<int> &x_locations, vector<int> &y_locations, const Mat &roiDiff, const TransitionRegion border, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y) {
    //used for detecting center point of robot
    int x_min = roiDiff.cols  + 1;
    int x_max = 0;
    int y_min = roiDiff.rows + 1;
    int y_max = 0;
    int robotPixelCount = 0;
    bool whiteDetectedInRow = false;
    bool whitePreviouslyFound = false;
    int spaceAfterWhiteCount = 0;
    for(int k = 0; k < roiDiff.cols - 1; k++){
        for(int j = 0; j < roiDiff.rows - 1; j++){
            whiteDetectedInRow = false;
            if(roiDiff.at<Vec3b>(Point(k,j)).val[0]/16 != 0 && roiDiff.at<Vec3b>(Point(k,j)).val[1]/16 != 0 && roiDiff.at<Vec3b>(Point(k,j)).val[2]/16 != 0){
                if(checkBorderingPixels(k, j, roiDiff)){
                    x_min = min(x_min, k);
                    y_min = min(y_min, j);
                    x_max = max(x_max, k);
                    y_max = max(y_max, j);
//                    robotRoi.at<Vec3b>(Point(k,j)) = Vec3b(66, 134, 244);
                    robotPixelCount++;
                    spaceAfterWhiteCount = 0;
                    whiteDetectedInRow = true;
                    whitePreviouslyFound = true;
//                        cout << "white" << endl;
                }
            }
        }
        //If we've found a robot at some point and have 10 rows without a white pixel detected, push shit
        if(whitePreviouslyFound && !whiteDetectedInRow){
            whiteDetectedInRow = false;
            spaceAfterWhiteCount++;
//                cout << spaceAfterWhiteCount << endl;
            if(spaceAfterWhiteCount == 10){
                whitePreviouslyFound = false;
                spaceAfterWhiteCount = 0;
                cout << "x_min: " << x_min << " x_max: " << x_max << endl;
                cout << "y_min: " << y_min << " y_max: " << y_max << endl;

                int x_center = (x_min + x_max)/2;
                int y_center = (y_min + y_max)/2;

                x_locations.push_back(border.xMin + x_center);
                y_locations.push_back(border.yMin + y_center);
                robot_sizes_x.push_back(x_max - x_min);
                robot_sizes_y.push_back(y_max - y_min);

                x_min = roiDiff.cols + 1;
                x_max = 0;
                y_max = 0;
                y_min = roiDiff.rows + 1;
            }
        }
    }
}

//modify this operator<<
/*
	friend ostream &operator<<(ostream &output, const State &S) {
		output << "Cost : " << S.m_cost << " " << S.m_stateInfo;
		return output;
	}
*/
