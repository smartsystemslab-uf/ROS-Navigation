//
// Created by afelder on 5/31/18.
//

#include "Aruco.h"
#include <cmath>

void Aruco::calibrate() {
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

void Aruco::testDetectByID(const string &filename, int id) {
    Mat frame = imread(filename);
//    int framesPerSecond = 2;

    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    cv::aruco::Dictionary markerDictionary = cv::aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    /*VideoCapture vid(0);

    if (!vid.isOpened()) {
        return -1;
    }*/

    //namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    vector<Vec3d> rotationVectors, translationVectors;


    cout << "Pre detect" << endl;
    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    //aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
    cout << "post detect" << endl;
    cout << markerIds.size() << endl;
    for (int i = 0; i < markerIds.size(); i++) {
//        aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
        aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    }
    cout << "Post draw" << endl;
    imshow("Webcam", frame);
    for(int i = 0; i < markerIds.size(); i++){
        //[0] is rotation
        //cout << markerIds[i] << ": " << rotationVectors[i][0] << " " << rotationVectors[i][1] << " " << rotationVectors[i][2] << endl;
        //cout << markerIds[i] << ": " << markerCorners[i][0] << " " << markerCorners[i][1] << " " << markerCorners[i][2] << " " << markerCorners[i][3] << "   " << rotationVectors[i][0] << endl;
//        if(markerIds[i] == id) {
            cout << markerIds[i] << "   " << getLocationMarker(markerCorners[i]) << "  Rotation: "
                 << getMarkerRotationDegrees(markerCorners[i]) << endl;
//        }
    }
    waitKey(0);
}

/**
 *
 * @param frames -- vector of N frames to check
 * @param id -- robot to find
 *
 *
 * @return -- vector containing corners of robot if found. Returns a vector with a single point (-1,-1) if not found.
 */
vector<Point2f> Aruco::detectByID(const vector<Mat> &frames, int id, int &index, int &frameIndex) const {
    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameters;

    cv::aruco::Dictionary markerDictionary = cv::aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    vector<Vec3d> rotationVectors, translationVectors;
    index = -1;
    bool found = false;
    //try to detect robot $id in each frame, continue until it's found
    for(int frame_ind = 0; frame_ind < frames.size(); frame_ind++){
        aruco::detectMarkers(frames[frame_ind], markerDictionary, markerCorners, markerIds);
        for(int id_ind = 0; id_ind < markerIds.size(); id_ind++){
            if(markerIds[id_ind] == id){
                found = true;
                index = id_ind;
                frameIndex = frame_ind;
                break;
            }
        }
        if(found) break;
        markerCorners.clear();
        markerIds.clear();

    }


//    //Debug spew
//    for(int i = 0; i < markerIds.size(); i++){
//       if(markerIds[i] == id) {
//           cout << "Found" << endl;
//            cout << markerIds[i] << "   " << getLocationMarker(markerCorners[i]) << "  Rotation: "
//                 << getMarkerRotationDegrees(markerCorners[i]) << endl;
//        }
//    }

    if(index == -1){
        markerCorners.resize(1);
        markerCorners[0].push_back(Point2f(-1,-1));
        index = 0;
    }
    return markerCorners[index];
}

void Aruco::cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients) {
    vector<vector<Point2f>> checkerBoardImageSpacePoints;
    getChessboardCorners(calibrationImages, checkerBoardImageSpacePoints, false);

    vector<vector<Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerBoardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    vector<Mat> rVectors, tVectors;
    distanceCoefficients = Mat::zeros(8, 1, CV_64F);

    calibrateCamera(worldSpaceCornerPoints, checkerBoardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool Aruco::saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) {
    ofstream outStream(name.c_str());
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

bool Aruco::loadCameraCalibration(string name) {
    ifstream inStream(name.c_str());
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
//                cout << cameraMatrix.at<double>(r, c) << "\n";
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
//                cout << distanceCoefficients.at<double>(r, c) << "\n";
            }
        }

        inStream.close();
        return true;

    }
    return false;
}

void Aruco::createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

void Aruco::getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults) {
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

void Aruco::setCalibrationSquareDimension(float calibrationSquareDimension) {
    Aruco::calibrationSquareDimension = calibrationSquareDimension;
}

void Aruco::setArucoSquareDimension(float arucoSquareDimension) {
    Aruco::arucoSquareDimension = arucoSquareDimension;
}

void Aruco::setChessboardDimensions(const Size &chessboardDimensions) {
    Aruco::chessboardDimensions = chessboardDimensions;
}

float Aruco::getCalibrationSquareDimension() const {
    return calibrationSquareDimension;
}

float Aruco::getArucoSquareDimension() const {
    return arucoSquareDimension;
}

const Size &Aruco::getChessboardDimensions() const {
    return chessboardDimensions;
}

Point2f Aruco::getLocationMarker(vector<Point2f> corners){
    float corner0x = corners[0].x;
    float corner0y = corners[0].y;
    float corner2x = corners[2].x;
    float corner2y = corners[2].y;
    return Point2f(((corner0x + corner2x)/2), ((corner0y+corner2y)/2));
//    float xDiffHalf = (abs(corner0x - corner2x))/2;
//    float yDiffHalf = (abs(corner0y - corner2y))/2;
//    float lesserNumberx = min(corner0x, corner2x);
//    float lesserNumbery = min(corner0y, corner2y);
////    float lesserNumberx = getLesser(corner0x, corner2x);
////    float lesserNumbery = getLesser(corner0y, corner2y);
//    return Point2f((lesserNumberx + xDiffHalf), (lesserNumbery + yDiffHalf));

}

//TODO: This can probably be made more efficient
double Aruco::getMarkerRotationDegrees(vector<Point2f> corners){
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
                return atan(yDiff/xDiff)*180/M_PI;
            }else{
                return -(atan(yDiff/xDiff)*180/M_PI);
            }
        }
    }else if(corner0x>corner1x){
        if(corner0y==corner1y){
            return 180.0;
        }else{
            if(corner0y<corner1y){
                return 180-(atan(yDiff/xDiff)*180/M_PI);
            }else{
                return -(180-(atan(yDiff/xDiff)*180/M_PI));
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
