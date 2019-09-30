//
// Created by andrew on 5/16/17.
//

#include "CeilingObjectDetection.h"
#include "ImageProcessing.h"
#include <climits>
#include <cmath>

using cv::imshow;
using cv::waitKey;
using cv::absdiff;
using cv::erode;
using cv::Size;
using cv::MORPH_RECT;
using cv::Vec3b;
using cv::Scalar;

vector<Point> CeilingObjectDetection::detectObjectCenter(const Mat &regionBinary, Mat &newImage, vector<int> &robotSizesX, vector<int> &robotSizesY) {
    Mat newImageBinary = ImageProcessing::performMeanShift(newImage, false);
    imwrite("ObjectDetectionSubtracted.jpg", newImageBinary);
//	cout << "Post mean shift of robotInView" << endl;
    Mat absDiff;
    int erodeSize = 3;
    Mat regionBinaryRead = cv::imread("subtractedImage.jpg",0);
//	cout << "pre conversion" << endl;
//	cout << "Region binary rows: " << regionBinary.rows << " Region binary cols: " << regionBinary.cols << " RegionBinary channels: " << regionBinary.channels() << endl;
//	cout << "newBinary rows: " << newImageBinary.rows << " newBinary cols: " << newImageBinary.cols << " newBinary channels: " << newImageBinary.channels() << endl;
	//if (regionBinary.channels() == 3) {
//		if(regionBinary.empty()){
//			cout << "empty region binary image. crash incoming" << endl;
//		}
//		cout << "Region Binary Type" <<  regionBinary.type() << endl;
//		cout << "Region Binary fixedType" << ((OutputArray)regionBinary).fixedType() << endl;
		//cv::cvtColor(regionBinary, regionBinary, cv::COLOR_BGR2GRAY);
	//}
//	cout << "pre absdiff" << endl;
    absdiff(regionBinaryRead, newImageBinary, absDiff);
//	cout << "Post absdiff" << endl;
    erode(absDiff, absDiff,
          getStructuringElement(MORPH_RECT,
                                Size(2 * erodeSize + 1, 2 * erodeSize + 1),
                                Point(erodeSize, erodeSize)));
//	cout << "Post erode" << endl;
    return getObjectXYTiled(absDiff, robotSizesX, robotSizesY, newImage, 19);
}

void CeilingObjectDetection::cleanVector(vector<int> &aVec, vector<int> &bVec, int distance, int replacementType) {
    assert(aVec.size() == bVec.size());
    if(aVec.size() == 0 && bVec.size() == 0) return;
    vector<int> atemp, btemp;
//    cout << aVec.size() << endl;
//    cout << bVec.size() << endl;
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


Mat CeilingObjectDetection::getClosestImage(const vector<Mat> &takenImages, const Mat &original){

    Mat bestImage;
    int bestCount = INT_MAX;
    for(int i = 0; i < takenImages.size();i++){
        Mat absDiffTemp;
        Mat takenOriginal = takenImages[i].clone();
        absdiff(takenImages[i],original, absDiffTemp);
        int zeroCount = countNonZeroDiv(absDiffTemp,1);
        if(zeroCount < bestCount){
            bestCount = zeroCount;
            bestImage = takenOriginal;
        }
    }
    return bestImage;

}

bool CeilingObjectDetection::valuesNear(int a, int b, double percentWithin) {
    return(a >= (b*(1-percentWithin)) && a <= (b*(1+percentWithin)));
}

bool CeilingObjectDetection::pixelsNear(int a, int b, int pixels) {
    return (a >= b-pixels && a <= b+pixels);
}

void CeilingObjectDetection::checkAndPush(int a, int b, vector<int> &aVec, vector<int> &bVec, int replacementType, int differenceMinimum) {
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

int CeilingObjectDetection::getIndexOfMaxValue(int *pixelCounts){
    int size = sizeof(pixelCounts) / sizeof(int);
    return (int)std::distance(pixelCounts, std::max_element(pixelCounts, pixelCounts+size));
}

int CeilingObjectDetection::countNonZeroDiv(Mat &image, int divValue) {
    int zeroPixels = 0;
    for(int i = 0; i < image.cols; i++){
        for(int j = 0; j < image.rows; j++){
            if(image.at<cv::Vec3b>(Point(i,j)).val[0]/divValue != 0 && image.at<cv::Vec3b>(Point(i,j)).val[1]/divValue != 0 && image.at<cv::Vec3b>(Point(i,j)).val[2]/divValue != 0){
                image.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(66, 134, 244);
                zeroPixels++;
            }
            else{
                image.at<cv::Vec3b>(Point(i,j)) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    return zeroPixels;
}

vector<Point> CeilingObjectDetection::getObjectXYTiled(const Mat &region, vector<int> &robotSizesX,
                                                       vector<int> &robotSizesY, const Mat &original, int rectSize) {
    vector<Point> returnPoints;
    vector<Rect> robotOutliningRects;
    Mat hlsRegion = region.clone();
	cout << "Pre conversion" << endl;
    cv::cvtColor(hlsRegion, hlsRegion, cv::COLOR_GRAY2BGR);
    cv::cvtColor(hlsRegion, hlsRegion, cv::COLOR_BGR2HLS);

    vector<int> yMins, yMaxs, xMins, xMaxs;
    int expansionAmount = 1;

	imwrite("ObjectDetectionHLS.jpg", hlsRegion);
    //cout << "Hls region rows: " << hlsRegion.rows << " Hls region cols: " << hlsRegion.cols << endl;
    for(int r = 0; r < hlsRegion.rows;r++){
        for(int c = 0; c < hlsRegion.cols; c++){
            //if we've already found this point, ignore it and start next loop
            bool cont = false;
            for(int s = 0; s < xMins.size();s++){
                if(c > xMins[s] && c < xMaxs[s] && r > yMins[s] && r < yMaxs[s]){
                    cont = true;
                    break;
                }
            }
            if(cont) continue;
            //if the pixel is white, try to see if it's a robot
            if(hlsRegion.at<Vec3b>(Point(c,r)).val[1] == 255){
//                cout << c << " " << r << endl
				//if(rectSize + c <= hlsRegion.cols || rectSize + r <= hlsRegion.rows){
					//rectSize = std::min(hlsRegion.cols - c, hlsRegion.rows - r) - 1;
				//}
				//if the detected point is too close to an edge, set rectSize to be [min(640 - c,480 - r) - 1] to prevent OOB issues
				rectSize = (rectSize + c > hlsRegion.cols || rectSize + r > hlsRegion.rows ? rectSize = std::min(hlsRegion.cols - c, hlsRegion.rows - r) - 1 : rectSize);
                Rect regionRect(c, r, rectSize, rectSize);
                bool expansionNeeded = true;
                while(expansionNeeded){
					//cout << "Region rect (x,y): (" << regionRect.x << ", " << regionRect.y << ")" << endl;
					//cout << "Expanding. regionRect width: " << regionRect.width << " regionRect height: " << regionRect.height << endl;
					//cout << "Hls region rows: " << hlsRegion.rows << " Hls region cols: " << hlsRegion.cols << endl;
                    expansionNeeded = false;
					//cout << "Check right" << endl;
                    if(checkRightBorderPixels(hlsRegion(regionRect), 255) && regionRect.x + regionRect.width  + expansionAmount < hlsRegion.cols - 1){
                        regionRect.width += expansionAmount;
                        expansionNeeded = true;
                    }
					//cout << "Finished right check" << endl;
                    //cout << "start left check" << endl;
					if(checkLeftBorderPixels(hlsRegion(regionRect), 255) && regionRect.x - expansionAmount > 0){
                        regionRect.x -= expansionAmount;
                        expansionNeeded = true;
                    }
					//cout << "end left check" << endl;
					//cout << "start top check" << endl;
                    if(checkTopBorderPixels(hlsRegion(regionRect), 255) && regionRect.y - expansionAmount > 0){
                        regionRect.y -= expansionAmount;
                        expansionNeeded = true;
                    }
					//cout << "end top check" << endl;
					//cout << "start bottom check" << endl;
                    if(checkBottomBorderPixels(hlsRegion(regionRect), 255) && regionRect.y + regionRect.height  + expansionAmount< hlsRegion.rows - 1){
                        regionRect.height += expansionAmount;
                        expansionNeeded = true;
                    }
					//cout << "end bottom check" << endl;
                }
				//cout << "expansion finished" << endl;
                if(regionRect.width >= rectSize + 1 && regionRect.height >= rectSize + 1) {
                    robotOutliningRects.push_back(regionRect);
                }
                yMins.push_back(regionRect.y);
                yMaxs.push_back(regionRect.y + regionRect.height);
                xMins.push_back(regionRect.x);
                xMaxs.push_back(regionRect.x + regionRect.width);

            }
        }
    }
    cout << "Size: " << robotOutliningRects.size() << endl;
    for(int i = 0; i < robotOutliningRects.size(); i++){
        cv::rectangle(hlsRegion, robotOutliningRects[i], CV_RGB(255, 0, 0), 2,8,0);
        int centerX = robotOutliningRects[i].x + (robotOutliningRects[i].width/2);
        int centerY = robotOutliningRects[i].y + (robotOutliningRects[i].height/2);
        if(returnPoints.size() == 0){
            robotSizesX.push_back(robotOutliningRects[i].width);
            robotSizesY.push_back(robotOutliningRects[i].height);
            returnPoints.push_back(Point(centerX, centerY));
        }
        else{
            for(int j = 0; j < returnPoints.size();j++){
                if(!pixelsNear(centerX, returnPoints[j].x,5) || !pixelsNear(centerY, returnPoints[j].y,5)){
//                if(centerX != returnPoints[j].x || centerY != returnPoints[j].y){
                    robotSizesX.push_back(robotOutliningRects[i].width);
                    robotSizesY.push_back(robotOutliningRects[i].height);
                    returnPoints.push_back(Point(centerX, centerY));
                    break;
                }
            }
        }

    }
    //detectEdges(original(robotOutliningRects[0]));
	vector<double> orientationAngles = detectOrientation(original, returnPoints, robotOutliningRects);
    //imshow("region", region);
    //waitKey(0);
    return returnPoints;

}

//if the right border pixels have luminous value >= l value, return true
bool CeilingObjectDetection::checkRightBorderPixels(const Mat &region, int lValue) {
	//cout << "Checking right" << endl;
    for(int i = 0; i < region.rows;i++){
        if(region.at<Vec3b>(Point(region.cols-1,i)).val[1] >= lValue){
            return true;
        }
    }
    return false;
}
bool CeilingObjectDetection::checkLeftBorderPixels(const Mat &region, int lValue) {
	//cout << "Checking left" << endl;
    for(int i = 0; i < region.rows;i++){
        if(region.at<Vec3b>(Point(0,i)).val[1] >= lValue){
            return true;
        }
    }
    return false;
}
bool CeilingObjectDetection::checkTopBorderPixels(const Mat &region, int lValue) {
	//cout << "Checking top" << endl;
    for(int i = 0; i < region.cols;i++){
        if(region.at<Vec3b>(Point(i,0)).val[1] >= lValue){
            return true;
        }
    }
    return false;
}
bool CeilingObjectDetection::checkBottomBorderPixels(const Mat &region, int lValue) {
	//cout << "Checking bottom" << endl;
    for(int i = 0; i < region.cols;i++){
        if(region.at<Vec3b>(Point(i,region.rows-1)).val[1] >= lValue){
            return true;
        }
    }
    return false;
}

vector<double> CeilingObjectDetection::detectOrientation(const Mat &robotRegion, vector<Point> robotCenterPoints, const vector<Rect> &robotRegionRects) {
    vector<double> angles;
    Mat robotBinary;
//    Mat c = robotRegion(robotRegionRect).clone();
//    circle(c, Point(orientationPoints[0].x, orientationPoints[0].y), 2, CV_RGB(255, 0, 0), 1, 8);
//    imshow("c", c);
//    waitKey(0);
    for(int i = 0; i < robotRegionRects.size();i++){
        robotBinary = ImageProcessing::performMeanShift(robotRegion(robotRegionRects[i]), false);
        vector<Point> orientationPoints = getObjectXYTiledNoSize(robotBinary, 1);
        int x = orientationPoints[i].x - (robotCenterPoints[i].x - robotRegionRects[i].x);
        int y = orientationPoints[i].y - (robotCenterPoints[i].y - robotRegionRects[i].y);
//        cout << x << " " << y << endl;
        //negative y because arctan assumes y increases as it goes up
        //in images it increases as it goes down
        double angle = atan2(-y,x) * (180/3.14159265);
        angles.push_back(angle);
        cout << "Orientation angle:  " <<  angle << endl;
    }
    return angles;
}

vector<Point> CeilingObjectDetection::getObjectXYTiledNoSize(const Mat &region, int rectSize) {
    vector<Point> returnPoints;
    vector<Rect> robotOutliningRects;
    Mat hlsRegion = region.clone();
    cv::cvtColor(hlsRegion, hlsRegion, cv::COLOR_GRAY2BGR);
    cv::cvtColor(hlsRegion, hlsRegion, cv::COLOR_BGR2HLS);

    vector<int> yMins, yMaxs, xMins, xMaxs;
    int expansionAmount = 1;

    for(int r = 0; r < hlsRegion.rows;r++){
        for(int c = 0; c < hlsRegion.cols; c++){
            //if we've already found this point, ignore it and start next loop
            bool cont = false;
            for(int s = 0; s < xMins.size();s++){
                if(c > xMins[s] && c < xMaxs[s] && r > yMins[s] && r < yMaxs[s]){
                    cont = true;
                    break;
                }
            }
            if(cont) continue;
            //if the pixel is white, try to see if it's a robot
            if(hlsRegion.at<Vec3b>(Point(c,r)).val[1] == 255){
//                cout << c << " " << r << endl;
                Rect regionRect(c, r, rectSize, rectSize);
                bool expansionNeeded = true;
                while(expansionNeeded){
                    expansionNeeded = false;
                    if(checkRightBorderPixels(hlsRegion(regionRect), 255) && regionRect.x + regionRect.width  + expansionAmount <= hlsRegion.cols - 1){
                        regionRect.width += expansionAmount;
                        expansionNeeded = true;
                    }
                    if(checkLeftBorderPixels(hlsRegion(regionRect), 255) && regionRect.x - expansionAmount >= 0){
                        regionRect.x -= expansionAmount;
                        expansionNeeded = true;
                    }
                    if(checkTopBorderPixels(hlsRegion(regionRect), 255) && regionRect.y - expansionAmount >= 0){
                        regionRect.y -= expansionAmount;
                        expansionNeeded = true;
                    }
                    if(checkBottomBorderPixels(hlsRegion(regionRect), 255) && regionRect.y + regionRect.height  + expansionAmount<= hlsRegion.rows - 1){
                        regionRect.height += expansionAmount;
                        expansionNeeded = true;
                    }
                }
                if(regionRect.width >= rectSize + 1 && regionRect.height >= rectSize + 1) {
                    robotOutliningRects.push_back(regionRect);
                }
                yMins.push_back(regionRect.y);
                yMaxs.push_back(regionRect.y + regionRect.height);
                xMins.push_back(regionRect.x);
                xMaxs.push_back(regionRect.x + regionRect.width);

            }
        }
    }
    cout << "Size: " << robotOutliningRects.size() << endl;
    for(int i = 0; i < robotOutliningRects.size(); i++){
        cv::rectangle(hlsRegion, robotOutliningRects[i], CV_RGB(255, 0, 0), 2,8,0);
        int centerX = robotOutliningRects[i].x + (robotOutliningRects[i].width/2);
        int centerY = robotOutliningRects[i].y + (robotOutliningRects[i].height/2);
        if(returnPoints.size() == 0){
            returnPoints.push_back(Point(centerX, centerY));
        }
        else{
            for(int j = 0; j < returnPoints.size();j++){
                if(!pixelsNear(centerX, returnPoints[j].x,5) || !pixelsNear(centerY, returnPoints[j].y,5)){
//                if(centerX != returnPoints[j].x || centerY != returnPoints[j].y){
                    returnPoints.push_back(Point(centerX, centerY));
                    break;
                }
            }
        }

    }
    return returnPoints;

}

void CeilingObjectDetection::detectEdges(const Mat &robotRegion) {
    Mat edgeImage = robotRegion.clone();
    Mat srcClone = robotRegion.clone();
    char *windowName = "edges";
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cvtColor(srcClone, srcClone, CV_BGR2GRAY);
    cv::blur(srcClone, edgeImage, cv::Size(3,3));
    cv::Canny(edgeImage, edgeImage, 200, 3, 3);
    Mat dst;
    dst.create(robotRegion.size(), robotRegion.type());
    dst = Scalar::all(0);
    robotRegion.copyTo(dst, edgeImage);
    imshow(windowName, dst);
    waitKey(0);
    exit(-1);
}
