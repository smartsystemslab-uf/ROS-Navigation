//
// Created by andrew on 3/7/17.
//

#include "RobotDetection.h"
#include <climits> 

using cv::Rect;
using cv::waitKey;

using std::cout;
using std::endl;

vector<Point> RobotDetection::detectLostRobot(/*const*/ Mat &originalImage, vector<Mat> &modifiedImages, vector<int> &robotSizesX,vector<int> &robotSizesY) {
    vector<Point> points;
//    Rect roi;
    int zeroPixels[4] = {0,0,0,0};
    vector<Mat> quadrants;
    Mat modifiedImage = modifiedImages[0].clone();
    

    cout << "Set modified Image" << endl;
    modifiedImage = getClosestImage(modifiedImages, originalImage, zeroPixels, quadrants);
	cout << "Finished setting modified Image" << endl;
    vector<int> indices;
    vector<int> partialIndices;
    
    //determine which quadrants may have a robot in them
    for(int i = 0;i < 4;i++){
//        cout << "zero pixels " << i << ": " << zeroPixels[i] << endl;
        //most of or entire robot in quadrant
        if(zeroPixels[i]/(double)((originalImage.rows/2)*(originalImage.cols/2)) > (double).01){
			cout << zeroPixels[i]/(double)((originalImage.rows/2)*(originalImage.cols/2)) << endl;
			cout << (originalImage.rows/2)*(originalImage.cols/2) << endl;
            indices.push_back(i);
		    cout <<"Indices: " <<  zeroPixels[i] << endl;
        }
        //part of robot in quadrant    
        else if(zeroPixels[i]/(double)((originalImage.rows/2)*(originalImage.cols/2)) > .005){
		   partialIndices.push_back(i);
	       cout << "Partial Indices: " << zeroPixels[i] << endl;
	    }
//		cout << zeroPixels[i] << endl;
    }
    cout << indices.size() << endl;
    cvtColor(modifiedImage, modifiedImage, cv::COLOR_GRAY2BGR);
//    vector<Point> intermediatePoints

    vector<Point> robotPoints;
    for(int i = 0;i<indices.size();i++){
        int index = indices[i];
        //imshow("diffed quadrant", quadrants[index]);
        vector<Point> intermediatePoints = getRobotXY(quadrants[index], robotSizesX, robotSizesY);
        cout << "Size: " << intermediatePoints.size() << endl;
        for(int j = 0; j < intermediatePoints.size();j++) {
            if (index == 1 || index == 3) intermediatePoints[j].x += originalImage.cols / 2;
            if (index == 2 || index == 3) intermediatePoints[j].y += originalImage.rows / 2;
            robotPoints.push_back(intermediatePoints[j]);
        }
//        circle(modifiedImage, Point(p.x, p.y), 4, CV_RGB(255, 0, 0), 3, 8);
    }

	//robots are on the border of quadrants
 	//check a horizontal and vertical section of original image
	if(partialIndices.size() > 1){
		cout << "checking cross" << endl;

        Mat originalImageROI;
        Mat modifiedImageROI;
        Mat roiDiff;


        //Erosion Prep -> erode seems to improve accuracy especially in case of rogue elements in image
        int erosion_type = 0;
        int erosion_size = 1;
        Mat element = getStructuringElement( erosion_type,
                                             cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                             Point( erosion_size, erosion_size ) );

        Rect horizontalRoi = Rect(0, originalImage.rows/4, originalImage.cols, originalImage.rows / 2);
        originalImageROI = originalImage(horizontalRoi).clone();
        modifiedImageROI = modifiedImage(horizontalRoi).clone();
        cvtColor(modifiedImageROI, modifiedImageROI, cv::COLOR_BGR2GRAY);
        absdiff(originalImageROI, modifiedImageROI, roiDiff);
        cvtColor(roiDiff, roiDiff, cv::COLOR_GRAY2BGR);
        erode( roiDiff, roiDiff, element);
        //zeroPixels array used to determine which quadrant the robot was in later
        cout << countNonZeroDiv16(roiDiff) << endl;
        
        vector<Point> crossPoints;
        crossPoints = getRobotXY(roiDiff, robotSizesX, robotSizesY);
        for(int i = 0; i < crossPoints.size(); i++){
            crossPoints[i].y += originalImage.rows/4;
            robotPoints.push_back(crossPoints[i]);
        }


        Rect verticalROI = Rect(originalImage.cols/4, 0, originalImage.cols/2, originalImage.rows);
        originalImageROI = originalImage(verticalROI).clone();
        modifiedImageROI = modifiedImage(verticalROI).clone();
        cvtColor(modifiedImageROI, modifiedImageROI, cv::COLOR_BGR2GRAY);
        absdiff(originalImageROI, modifiedImageROI, roiDiff);
        cvtColor(roiDiff, roiDiff, cv::COLOR_GRAY2BGR);
        erode( roiDiff, roiDiff, element);
        //zeroPixels array used to determine which quadrant the robot was in later
        cout << countNonZeroDiv16(roiDiff) << endl;

        crossPoints.clear();
        crossPoints = getRobotXY(roiDiff, robotSizesX, robotSizesY);
        for(int i = 0; i < crossPoints.size(); i++){
            crossPoints[i].x += originalImage.cols/4;
            robotPoints.push_back(crossPoints[i]);
        }
	}
	for(int i = 0; i < robotPoints.size();i++){
		cout << i << endl; 
        circle(modifiedImage, Point(robotPoints[i].x, robotPoints[i].y), 4, CV_RGB(255, 0, 0), 3, 8);
    }
	imwrite("detectedBots.jpg", modifiedImage);

    //imshow("Robots", modifiedImage);
    //waitKey(0);
    //get the X,Y coordinates of the robot relative to the quadrant
//    Point robotPoint = getRobotXY(quadrants[index], robotSizesX, robotSizesY);
    
    //add extra to local x, local y where needed to get global x, global y
    /**
     *                   cols/2
     * ___________________|_________________________
     *|                    |                         |
     *|                    |                         |
     *|      index 0       |      index 1            |
     *|      local x       |  local x + cols/2       |
     *|      local y       |      local y            |
     * ----------------------------------------------|  rows/2
     *|                    |                         |
     *|     index 2        |       index 3           | 
     *|     local x        |  local x + cols/2       | 
     *|  local y + rows/2  |  local y + rows/2       |    
     * ----------------------------------------------
     */
//    if(index == 1 || index == 3) robotPoint.x += originalImage.cols/2;
//    if(index == 2 || index == 3) robotPoint.y += originalImage.rows/2;

//    points.push_back(robotPoint);
    return robotPoints;
}

int RobotDetection::countNonZeroDiv16(Mat &image) {
    //we're doing a div by 16 since normal countNonZero breaks like hell for us
    //cvtColor(image, image, cv::COLOR_GRAY2BGR);
    int zeroPixels = 0;
    for(int i = 0; i < image.cols; i++){
        for(int j = 0; j < image.rows; j++){
            if(image.at<cv::Vec3b>(Point(i,j)).val[0]/16 != 0 && image.at<cv::Vec3b>(Point(i,j)).val[1]/16 != 0 && image.at<cv::Vec3b>(Point(i,j)).val[2]/16 != 0){
                image.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(66, 134, 244);
                zeroPixels++;
            }
            else{
                image.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(0, 0, 0);
            }
        }
    } 
   // cvtColor(image, image, cv::COLOR_BGR2GRAY);
//	if(image.cols == 640){
 	//	imshow("region", image);
	   // waitKey(0);
	//}
    return zeroPixels;
}

//there's probably a better way to do this
int RobotDetection::maxIndex(int pixelCounts[]){
    int m1 = std::max(pixelCounts[0],pixelCounts[1]);
    int m2 = std::max(pixelCounts[2],pixelCounts[3]);
    int maxFinal = std::max(m1, m2);
    if(maxFinal == pixelCounts[0]) return 0;
    if(maxFinal == pixelCounts[1]) return 1;
    if(maxFinal == pixelCounts[2]) return 2;
    if(maxFinal == pixelCounts[3]) return 3;
    return -1;
   
}

//also gets the size of the robot
//bear with me, I know this is a bit janky
vector<Point> RobotDetection::getRobotXY(const Mat &region, vector<int> &robotSizesX,vector<int> &robotSizesY){
    vector<Point> returnPoints;
    vector<int> yMins, yMaxs, xMins, xMaxs;
    int robotSize = 60; // THIS IS ONLY TEMPORARILY HARDCODED
    bool whiteFound = false;
    bool whiteFoundOnColumn = false;
    int blackBreakCounter = 0;
    int pixelDistanceCounter = 0;
    int xMinCurr = region.cols, xMaxCurr = 0, yMinCurr = region.rows, yMaxCurr = 0;
    if(region.cols == 640){
        cout << region.cols << " " << region.rows <<  " " << region.size << endl;
        //imshow("region", region);
        //waitKey(0);        
    }

	cout << region.cols << " " << region.rows << endl;
    for(int i = 0; i < region.cols; i++){
        for(int j = 0; j < region.rows; j++){
            //if we haven't found a non-black pixel on row j in ~robotsize pixels, break to the next row
            if(blackBreakCounter > robotSize/3){
                checkAndPush(yMinCurr, yMaxCurr, yMins, yMaxs, MIN_MAX, robotSize/3);
                yMaxCurr = 0;
                yMinCurr = region.rows;
//                cout << "max: " << xMaxCurr << " min: " << xMinCurr << endl;
                checkAndPush(xMinCurr, xMaxCurr, xMins, xMaxs, MIN_MAX, robotSize/3);
                whiteFound = false;
                blackBreakCounter = 0;
            }
            //if pixel isn't black reset break counter, and set a white pixel to have been found
            if(region.at<cv::Vec3b>(Point(i,j)).val[0]/16 != 0 && region.at<cv::Vec3b>(Point(i,j)).val[1]/16 != 0 && region.at<cv::Vec3b>(Point(i,j)).val[2]/16 != 0){
//                region.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(66, 134, 244);
                whiteFound = true;
                whiteFoundOnColumn = true;
                blackBreakCounter = 0;
                xMinCurr = std::min(xMinCurr,i);
                xMaxCurr = std::max(xMaxCurr, i);
                yMinCurr = std::min(yMinCurr,j);
                yMaxCurr = std::max(yMaxCurr, j);
            }
            //if we have found a white pixel on row j, and the current pixel is black, increment counter for potential breaking purposes    
            else if(whiteFound) {
                blackBreakCounter++;
            }
        }
        //make sure we don't increment on a row where we broke after finding a white pixel
        if(!whiteFoundOnColumn) {
            pixelDistanceCounter++;
        }
        //if we haven't found a robot in ~robotsize columns, push the current one and start over from current point
        if(pixelDistanceCounter > robotSize/3/*MAGIC NUMBERS -> Robot's size*/){
            if(xMaxCurr == 0 && xMinCurr == region.cols){
                pixelDistanceCounter = 0;
            }
            else {
//                checkAndPush(xMinCurr, xMaxCurr, xMins, xMaxs, MIN_MAX, robotSize / 3);
                xMaxCurr = 0;
                xMinCurr = region.cols;
                pixelDistanceCounter = 0;
            }
        }
        whiteFoundOnColumn = false;
        whiteFound = false;
    }
    cout << "before clean" << endl;
//    cleanVector(yMaxs, robotSize/3, MAX);
    cleanVector(yMins, yMaxs, robotSize/3, MIN_MAX);
//    cleanVector(yMins, yMaxs, robotSize/3, MIN_MAX);
//    cleanVector(yMaxs, robotSize/3, MAX);
//    cleanVector(yMins, robotSize/3, MIN);
//    cleanVector(xMaxs, robotSize/3, MAX);
    cleanVector(xMins, xMaxs, robotSize/3, MIN_MAX);

    for(int i = 0; i < yMaxs.size(); i++){
        cout << yMaxs[i] << " ";
    }
    cout << endl;
    for(int i = 0; i < yMins.size(); i++){
        cout << yMins[i] << " ";
    }
    cout << endl;
    cout << xMaxs.size() << " " << xMins.size() << " " << yMaxs.size()  << " " << yMins.size() << endl;
    //assert((yMaxs.size() == yMins.size() && yMaxs.size() == xMaxs.size() && yMaxs.size() == xMins.size()) || (yMaxs.size() == yMins.size() && yMaxs.size() == 1) || (xMaxs.size() == xMins.size() && xMaxs.size() == 1));
//    for(int i = 0; i < yMaxs.size();i++){
//        returnPoints.push_back(Point((xMaxs[i]+xMins[i])/2,(yMaxs[i]+yMins[i])/2));
//    }
    if(xMaxs.size() == 1 || yMaxs.size() == 1){
        for(int i = 0;i < yMaxs.size();i++){
            for(int j = 0; j < xMaxs.size();j++){
                returnPoints.push_back(Point((xMaxs[j]+xMins[j])/2,(yMaxs[i]+yMins[i])/2));
            }
        }
    }
    
    //robotSizesX.push_back(xMaxCurr-xMinCurr);
    //robotSizesY.push_back(yMaxCurr-yMinCurr);
	for (int i = 0; i < xMaxs.size(); i++) {
		robotSizesX.push_back(xMaxs[i] - xMins[i]);
	}
	for (int i = 0; i < yMaxs.size(); i++) {
		robotSizesY.push_back(yMaxs[i] - yMins[i]);
	}
    return returnPoints;
}

void RobotDetection::checkAndPush(int a, int b, vector<int> &aVec, vector<int> &bVec, int replacementType, int differenceMinimum) {
    assert(aVec.size() == bVec.size());
    bool valuesChanged = false;
    if(pixelsNear(a, b, differenceMinimum)){
        return;
    }
    if(aVec.size() == 0){
        aVec.push_back(a);
        bVec.push_back(b);
        return;
    }
    for(int i = 0; i < aVec.size();i++){
//        if(valuesNear(a, aVec[i],.025) && valuesNear(b, bVec[i], .025)){
        if(pixelsNear(a, aVec[i], 10) && pixelsNear(b, bVec[i], 10)){
            valuesChanged = true;
            if(replacementType == MAX){
                aVec[i] = std::max(aVec[i], a);
                bVec[i] = std::max(bVec[i], b);
            }
            else if(replacementType == MIN){
                aVec[i] = std::min(aVec[i], a);
                bVec[i] = std::min(bVec[i], b);
            }
            else if(replacementType == REPLACE_REGARDLESS){
                aVec[i] = a;
                bVec[i] = b;
            }
            else if(replacementType == MIN_MAX){
//                cout << bVec[i] << " " << b << endl;
                aVec[i] = std::min(aVec[i], a);
                bVec[i] = std::max(bVec[i], b);
            }
            else{
                cout << "Invalid replacementType" << endl;
                exit(-1);
            }
        }
    }
//    cout << endl;
    if(!valuesChanged){
        aVec.push_back(a);
        bVec.push_back(b);
    }
}

bool RobotDetection::valuesNear(int a, int b, double percentWithin) {
    return(a >= (b*(1-percentWithin)) && a <= (b*(1+percentWithin)));
}

bool RobotDetection::pixelsNear(int a, int b, int pixels) {
    return (a >= b-pixels && a <= b+pixels);
}

void RobotDetection::cleanVector(vector<int> &aVec, vector<int> &bVec, int distance, int replacementType) {
    assert(aVec.size() == bVec.size()); 
	if(aVec.size() == 0 && bVec.size() == 0) return;
    vector<int> atemp, btemp;
    cout << aVec.size() << endl;
    cout << bVec.size() << endl;
    atemp.push_back(aVec[0]);
    btemp.push_back(bVec[0]);
    bool valuesChanged = false;
    for(int i = 1; i < aVec.size();i++){
        for(int j = 0; j< atemp.size();j++){
            if(pixelsNear(aVec[i], atemp[j], distance) || pixelsNear(bVec[i], btemp[j], distance)){
                valuesChanged = true;
                if(replacementType == MAX){
                    atemp[j] = std::max(atemp[j], aVec[i]);
                    btemp[j] = std::max(btemp[j], bVec[i]);
                    break;
                }
                else if(replacementType == MIN){
                    atemp[j] = std::min(atemp[j], aVec[i]);
                    btemp[j] = std::min(btemp[j], bVec[i]);
                    break;
                }
                else if(replacementType == REPLACE_REGARDLESS){
                    atemp[j] = aVec[i];
                    btemp[j] = bVec[i];
                    break;
                }
                else if(replacementType == MIN_MAX){
                    atemp[j] = std::min(aVec[i], atemp[j]);
                    btemp[j] = std::max(bVec[i], btemp[j]);
                    break;
                }    
                else{
                    cout << "Supported replacement types: MIN, MAX, REPLACE_REGARDLESS" << endl;
                    return;
                }
            }
        }
        if(!valuesChanged){
            atemp.push_back(aVec[i]);
            btemp.push_back(bVec[i]);
        }
        valuesChanged = false;
    }
    aVec = atemp;
    bVec = btemp;
}


int countNonZeroTest(Mat &image) {
    //we're doing a div by 16 since normal countNonZero breaks like hell for us
    cvtColor(image, image, cv::COLOR_GRAY2BGR);
    int zeroPixels = 0;
    for(int i = 0; i < image.cols; i++){
        for(int j = 0; j < image.rows; j++){
            if(image.at<cv::Vec3b>(Point(i,j)).val[0] != 0 && image.at<cv::Vec3b>(Point(i,j)).val[1] != 0 && image.at<cv::Vec3b>(Point(i,j)).val[2] != 0){
//                image.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(66, 134, 244);
                zeroPixels++;
            }
        }
    }
    cvtColor(image, image, cv::COLOR_BGR2GRAY);

    return zeroPixels;
}


void drawOrange(Mat &image){
    cvtColor(image, image, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < image.cols; i++){
        for(int j = 0; j < image.rows; j++){
            if(image.at<cv::Vec3b>(Point(i,j)).val[0] != 0 && image.at<cv::Vec3b>(Point(i,j)).val[1] != 0 && image.at<cv::Vec3b>(Point(i,j)).val[2] != 0){
                image.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(66, 134, 244);
            }
        }
    }
}

void RobotDetection::testNoiseReduction(const Mat &original, const Mat &modified) {
    Mat result;

    int erosion_type = 0;
    int erosion_size = 1;
    Mat element = getStructuringElement( erosion_type,
                                         cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
    
    Mat drawnOn;
    absdiff(original, modified, result);
//    imshow("original", modified);
//    waitKey(0);
    
    //imshow("original", result);
    //waitKey(0);
    imwrite("originalDiff.jpg", result);
    
    drawnOn = result.clone();
    drawOrange(drawnOn);
    //imshow("no reduction", drawnOn);
    //waitKey(0);
    imwrite("noReduction.jpg", drawnOn);
    
    cout << "Pre erode " << countNonZero(result) << endl;

    erode( result, result, element);
    
    drawnOn = result.clone();
    drawOrange(drawnOn);
    //imshow("Post erode", drawnOn);
    //waitKey(0);
    imwrite("PostErosion.jpg", drawnOn);
    
    cout << "Post erode " << countNonZero(result) << endl;

    cout << "Post div 16 " << countNonZeroDiv16(result) << endl;

    drawnOn = result.clone();
    drawOrange(drawnOn);
    //imshow("Post Div", drawnOn);
    //waitKey(0);
    imwrite("PostDiv.jpg", drawnOn);

}

Mat RobotDetection::getClosestImage(const vector<Mat> &takenImages, const Mat &original, int zeroPixelCounts[3], vector<Mat> &bestQuadrants){

    Rect roi;
    Mat originalImageQuad;
    Mat modifiedImageQuad;
    Mat modifiedImage;
    Mat bestImage;
    Mat roiDiff;
    int zeroPixels[4] = {0,0,0,0};
    int bestZeroSum = INT_MAX;
    vector<Mat> quadrants;

    //Erosion Prep -> erode seems to improve accuracy especially in case of rogue elements in image
    int erosion_type = 0;
    int erosion_size = 1;
    Mat element = getStructuringElement( erosion_type,
                                         cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );

    for(int i = 0; i < takenImages.size();i++) {
	quadrants.clear();
        modifiedImage = takenImages[i];
	cvtColor(modifiedImage, modifiedImage, cv::COLOR_BGR2GRAY);
        //Start with top left
        roi = Rect(0, 0, original.cols / 2, original.rows / 2);
        originalImageQuad = original(roi).clone();
        modifiedImageQuad = modifiedImage(roi).clone();    
	absdiff(originalImageQuad, modifiedImageQuad, roiDiff);
        cvtColor(roiDiff, roiDiff, cv::COLOR_GRAY2BGR);
       erode( roiDiff, roiDiff, element);
        //save diff'd quadrant to actually get the robot later in this function. Currently just seeing where robots may be
        quadrants.push_back(roiDiff);
        //zeroPixels array used to determine which quadrant the robot was in later
        zeroPixels[0] = countNonZeroDiv16(roiDiff);
	//imshow("topLeft", roiDiff);
	//waitKey(0);

        //top right
        roi = Rect(original.cols / 2, 0, original.cols / 2, original.rows / 2);
        originalImageQuad = original(roi).clone();
        modifiedImageQuad = modifiedImage(roi).clone();
cout << originalImageQuad.channels() << " " << modifiedImageQuad.channels() << endl;
        absdiff(originalImageQuad, modifiedImageQuad, roiDiff);
        cvtColor(roiDiff, roiDiff, cv::COLOR_GRAY2BGR);
      erode( roiDiff, roiDiff, element );
        quadrants.push_back(roiDiff);
        zeroPixels[1] = countNonZeroDiv16(roiDiff);
	//imshow("topRight", roiDiff);
	//waitKey(0);

        //bottom left
        roi = Rect(0, original.rows / 2, original.cols / 2, original.rows / 2);
        originalImageQuad = original(roi).clone();
        modifiedImageQuad = modifiedImage(roi).clone();
        absdiff(originalImageQuad, modifiedImageQuad, roiDiff);
        cvtColor(roiDiff, roiDiff, cv::COLOR_GRAY2BGR);
       erode( roiDiff, roiDiff, element );
        quadrants.push_back(roiDiff);
        zeroPixels[2] = countNonZeroDiv16(roiDiff);
	//imshow("bottomLeft", roiDiff);
	//waitKey(0);

        //bottom right
        roi = Rect(original.cols / 2, original.rows / 2, original.cols / 2, original.rows / 2);
        originalImageQuad = original(roi).clone();
        modifiedImageQuad = modifiedImage(roi).clone();
        absdiff(originalImageQuad, modifiedImageQuad, roiDiff);
        cvtColor(roiDiff, roiDiff, cv::COLOR_GRAY2BGR);
       erode( roiDiff, roiDiff, element );
        quadrants.push_back(roiDiff);
        zeroPixels[3] = countNonZeroDiv16(roiDiff);
	//imshow("bottomRight", roiDiff);
	//waitKey(0);       

        if(zeroPixels[0] + zeroPixels[1] + zeroPixels[2] + zeroPixels[3] < bestZeroSum){
            bestZeroSum = zeroPixels[0] + zeroPixels[1] + zeroPixels[2] + zeroPixels[3];
            zeroPixelCounts[0] = zeroPixels[0];
            zeroPixelCounts[1] = zeroPixels[1];
            zeroPixelCounts[2] = zeroPixels[2];
            zeroPixelCounts[3] = zeroPixels[3];
            bestImage = modifiedImage;
	    bestQuadrants = quadrants;
        }
    }
    return bestImage;

}
