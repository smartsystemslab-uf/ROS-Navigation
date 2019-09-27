
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <list>
#include <cmath>
#include <vector>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <errno.h>

#include <pthread.h>




using namespace std;
using namespace cv;

#define portno 60000 // port number to use
#define buf_size 4096 // max file size to receive

#define SEGMENT_THRESHOLD .3

/*
 * DIV_BY_16 and DIV_BY_8 should not be defined at the same time.
 * either one or none should be defined.
 * if neither is active, then the rgb values will range 0-255
 * div by 8 - range is 0-31
 * div by 15 - range is 0-15
 */

//#define DIV_BY_16 true
#define DIV_BY_8 true
//#define THRESHOLD 75
#define RUNTIME_SEQUENCE
// #define USE_HSV true

#ifdef DIV_BY_16
int GroundColors[16][16][16] = {0};
#elif (DIV_BY_8)
int GroundColors[32][32][32] = {0};
int PossibleGroundColors[32][32][32] = {0};
int CurrentFrameColors[32][32][32] = {0};
int maxDim = 32;
#else
int GroundColors[255][255][255] = {0};
#endif

unsigned int pinDelay = 280;
unsigned int pinDelayFwd = 265;
int fwdPin = 1;
int rightPin = 23;
int leftPin = 24;

Size frameSize(640, 480);

const int MAX_SEGMENTS = 14;
Mat groundImage;
Mat outputImage;
Mat comparisonImage;
Mat frame;
Mat RGBOutputFrame;
int trapezoidBaseLeft_x;
int trapezoidBaseRight_x;
int trapezoidTopLeft_x;
int trapezoidTopRight_x;
int trapezoidMaxHeight;
int trapezoidMidpoint;

VideoCapture cap;

list<Vec3b> imageColors;

Point3_<uchar>* RGB;
int noiseThresholdCount = 0;

int groundPix = 0;
int obstaclePix = 0;

int maxR, minR, maxG, minG, maxB, minB;

//GoPiGo myRobot;	//create GoPiGo object

int rc; // holds return code of system calls
int sock; // socket desciptor
int desc; // file descriptor for socket

char const* fr_name = "./path.txt"; // path to file
vector<u_char> pathVector; // Vector for storing path informations


pair<int, int> segments[MAX_SEGMENTS];
/*
struct Segment {
	int obstacleCount = 0;
	int totalPixel = 0;

}
*/



int nextStep =0;
vector<u_char> pathCorrection;
vector<u_char> pathPrediction;

int numStepsForward = 0;
//int lastNumStepsForward;
//int nextPrediction = 0;
int nextNumStepsForward = 0;
u_char nextTurn = '0';
u_char nextCorrection = '0';
u_char nextEvasion = '0';
int numStepsTillTurn = 0;
int numTurns =0;
int generalDirection = 0;
bool moving = false;
bool useNearFieldNav = false;
bool onPrediction = false;
bool onCorrection = false;
bool pathHasTurns = false;


bool getNextTurn(){
    int i = nextStep;
    numStepsTillTurn = 0;
    while( i < pathVector.size() -1){
        if(pathVector.at(i) != '0') {
            nextTurn = pathVector.at(i);
            return true;
        }
        i++;
        numStepsTillTurn++;
    }
}

bool assessPath(){
    bool pathHasTurns = getNextTurn();
    if( pathHasTurns ){
        int i = nextStep + numStepsTillTurn;
        while( i < pathVector.size()) {
            if (pathVector.at(i) == '1') {
                generalDirection++;
                numTurns++;
            } else if (pathVector.at(i) == '7') {
                generalDirection--;
                numTurns++;
            }
            else if (pathVector.at(i) == '0') {
                nextNumStepsForward++;

            }
            i++;

        }
    }
}

bool fwdPredictionTake1(){
    numStepsForward = 0;

    nextTurn = '0';
    numStepsTillTurn = 0;
    int generalDirection = 0;
    int i = nextStep;
    while (pathVector.at(i) == '0') {

        numStepsForward++;
        numStepsTillTurn++;
        i++;

    }
    if( numStepsForward == pathVector.size() - 1 - nextStep )
        return true;
    else {
        if (numStepsTillTurn < 8) {

            useNearFieldNav = true;
        } else
            useNearFieldNav = false;
        nextTurn = pathVector.at(i);
//    if (nextTurn == '1')
//        nextCorrection = '7';
//    else if (nextTurn == '7')
//        nextCorrection = '1';
        // lastNumStepsForward = numStepsForward;
        nextNumStepsForward = 0;
        //set general direction by cycling through rest
        //of path and addingfor right turns, subtracting
        //for left turns...
        while (i < pathVector.size()) {
            if (pathVector.at(i) == '1')
                generalDirection++;
            else if (pathVector.at(i) == '7')
                generalDirection--;
            else if (pathVector.at(i) == '0')
                nextNumStepsForward++;
            i++;
        }
        if (nextTurn == '1' && generalDirection < 0) {
            nextCorrection = nextTurn;
            nextEvasion = '7';
        } else {
            nextCorrection = '7';
            nextEvasion = '1';
        }
        if (nextTurn == '7' && generalDirection > 0) {
            nextCorrection = nextTurn;
            nextEvasion = '1';
        } else {
            nextCorrection = '1';
            nextEvasion = '7';
        }
    }
    return false;
}



void processRGB(int x, int y){
    //loop over the image and store the R values into a map
    for(int i = 0;i < x; i++){
        // cout << "Processing column " << i << " of " << x << endl;
        for(int j = 0;j < y; j++){
            RGB = groundImage.ptr<Point3_<uchar> >(j,i);
            // int totalValue;
#ifdef DIV_BY_16
            // totalValue = ((RGB->z/16) * 10000) + ((RGB->y/16) * 100) + (RGB->x/16);
			GroundColors[RGB->z/16][RGB->y/16][RGB->x/16]++;

#elif (DIV_BY_8)
            // totalValue = ((RGB->z/8) * 10000) + ((RGB->y/8) * 100) + (RGB->x/8);
            GroundColors[RGB->z/8][RGB->y/8][RGB->x/8]=1;
            PossibleGroundColors[RGB->z/8][RGB->y/8][RGB->x/8] = 1;


#else
            // totalValue = RGB->z * 1000000 + RGB->y * 1000 + RGB->x;
			GroundColors[RGB->z][RGB->y][RGB->x]++;


#endif
        }
    }
}




            //  0 ,                 1          ,     2         ,     3          ,     4         ,       5       ,       6               ,       7               ,           8       ,       9              ,       10           ,       11          ,       12            ,       13
enum segment{LEFT_RECTANGLE = 0, LEFT_TRIANGLE, LEFT_TRAPEZOID, RIGHT_TRAPEZOID, RIGHT_TRIANGLE, RIGHT_RECTANGLE, LEFT_TRAPEZOID_3_4THS, RIGHT_TRAPEZOID_3_4THS, LEFT_TRAPEZOID_HALF, RIGHT_TRAPEZOID_HALF, LEFT_TURN_RECTANGLE, LEFT_TRAPEZOID_BASE, RIGHT_TRAPEZOID_BASE, RIGHT_TURN_RECTANGLE};


/*                     x=260           x=379
 * _____________________X_______________X________________________
 *          |           |   2   |   3   |           |           |
 *          |       ____|_______|_______|____       |           |
 *    0     |   1   |           |           |   4   |     5     |
 *          |       |   6       |     7     |       |           |
 *          |   ____|___________|___________|____   |           |
 *          |   |               |               |   |           |
 *          |   |      8        |      9        |   |           |
 *          |   |               |               |   |           |
 * _________|___|_______________|_______________|___|___________|
 *          |                   |                   |           |
 *    10    |        11         |      12           |    13     |
 *          |                   |                   |           |
 *
 *
 * // THE FOLLOWING DESCRIPTIONS ARE NOT ACCURATE CURRENTLY.
 * //EACH SECTION IS INDEPENDENTLY NUMBERED, STARTING AT ZERO
 * AND GOING TO THIRTEEN.  UGH.
 *
 * 3 and 4 include all four of the rectangles in the left and right
 * half of the robot forward path roi
 *
 *  7 and 8 contain the three largest rectangles for the left and right
 *  half of the robot forward path roi
 *
 *  9 and 10 contain the two largest rectangles for the left and right
 *  half of the robot forward path roi
 *
 *  12 and 13 are the bottom rectangles directly in front of the robot
 *
 *  11 is the bottom left area, used for checking that left turns are
 *  clear
 *
 *  14 is the bottom right area, used for checking that right turns
 *  are clear
 *
 *
 *
 *
 *
 *
 *
 */




int AssignSegment(int r, int c, int boundary_right, int boundary_left){
//    cout << "br: " << boundary_right << " " << "bl: " << boundary_left << endl;
//    cout << "r : " << r << "    c : " << c << endl;
//    cout << "trapezoidBaseLeft_x : " << trapezoidBaseLeft_x << "   boundary_left : " << boundary_left << endl;

    if(c <= trapezoidBaseLeft_x && r < 400){
        return LEFT_RECTANGLE;
    }
    else if(c <= trapezoidBaseLeft_x && r >= 400){
        return LEFT_TURN_RECTANGLE;
    }
    else if(c > trapezoidBaseLeft_x && c < boundary_left){
        return LEFT_TRIANGLE;
    }
    else if(c >= boundary_left && c <= trapezoidMidpoint && r < 295){
        return LEFT_TRAPEZOID;
    }
    else if(c >= boundary_left && c <= trapezoidMidpoint && r < 340){
        return LEFT_TRAPEZOID_3_4THS;
    }
    else if(c >= boundary_left && c <= trapezoidMidpoint && r < 400){
        return LEFT_TRAPEZOID_HALF;
    }
    else if(c >= boundary_left && c <= trapezoidMidpoint && r >= 400){
        return LEFT_TRAPEZOID_BASE;
    }
    else if(c > trapezoidMidpoint && c <= boundary_right && r < 295){
        return RIGHT_TRAPEZOID;
    }
    else if(c > trapezoidMidpoint && c <= boundary_right && r < 340){
        return RIGHT_TRAPEZOID_3_4THS;
    }
    else if(c > trapezoidMidpoint && c <= boundary_right && r < 400){
        return RIGHT_TRAPEZOID_HALF;
    }
    else if(c > trapezoidMidpoint && c <= boundary_right && r >= 400){
        return RIGHT_TRAPEZOID_BASE;
    }
    else if(c > boundary_right && c <= trapezoidBaseRight_x){
        return RIGHT_TRIANGLE;
    }
    else if(c > trapezoidBaseRight_x && r < 400) {
        return RIGHT_RECTANGLE;
    }
    else if(c > trapezoidBaseRight_x && r >= 400)
        return RIGHT_TURN_RECTANGLE;
}
void processPixelRGB(int x, int y){
    // for(int i = 0;i < x; i++){
    // cout << "Processing column " << i << " of " << x << endl;
    // for(int j = 0;j < y; j++){
    RGB = groundImage.ptr<Point3_<uchar> >(y,x);
    // int totalValue;
#ifdef DIV_BY_16
    // totalValue = ((RGB->z/16) * 10000) + ((RGB->y/16) * 100) + (RGB->x/16);
			GroundColors[RGB->z/16][RGB->y/16][RGB->x/16]++;

#elif (DIV_BY_8)
    int zRGB = RGB->z/8;
    int yRGB = RGB->y/8;
    int xRGB = RGB->x/8;

    // cout << "!!! added an entry to the lut " << endl;

    GroundColors[zRGB][yRGB][xRGB]=1;
    PossibleGroundColors[zRGB][yRGB][xRGB]=1;


#else
    // totalValue = RGB->z * 1000000 + RGB->y * 1000 + RGB->x;
			GroundColors[RGB->z][RGB->y][RGB->x]++;


#endif
}

void processRectangleGround(Point top_left, Point top_right, Point bottom_left, Point bottom_right){

    float top_row = top_left.y;
    float bottom_row = bottom_left.y;

    float left_y = bottom_left.y - top_left.y;
    float left_x = bottom_left.x - top_left.x;

    float right_y = bottom_right.y- top_right.y;
    float right_x = bottom_right.x - top_right.x;


    float slope_right = right_y/right_x;
    float slope_left = left_y/left_x;



    // slope_right = (slope_right < .8) ? .8 : slope_right;
    // slope_left = (slope_left < .8) ? .8 : slope_left;
    // slope_right *= 1.3;
    // slope_left = -slope_right;

//    float boundary_left = top_left.x;
//    float boundary_right = top_right.x;

    float boundary_left = 169;
    float boundary_right = 470;
    float rect_top = 340;

    // previously: int r = top_row
    //changed it to rect_top to only process the bottom six segments of the
    // robot path roi...
    for(int r = rect_top; r < bottom_row; r++){
        // cout << "Processing row " << r << " of " << bottom_row << endl;
        for(int c = boundary_left; c < (int)boundary_right; c++){

            // uncomment the next line for production code
            processPixelRGB(c,r);
        }


        if (r == 295) {
            boundary_left = 221; boundary_right = 418; //rect_top = 340;
            //cout << " top rectangle at y = 295, x = " << c << endl;
        }
        if( r == 340 ) {
            boundary_left = 169; boundary_right = 470; //rect_top = 400;
          //  cout << " rectop = " << rect_top << endl;
            //  cout << " top rectangle at y = 340, x = " << c << endl;
        }
        if( r == 400 ) {
            boundary_left = 100; boundary_right = 540; //rect_top = r;
            //cout << " top rectangle at y = 400, x = " << c << endl;
        }

//
//        boundary_left += 1/slope_left;
//        boundary_right += 1/slope_right;
    }


    // cout << "before for loop";
    // for(int r = groundImage.rows/2; r < groundImage.rows; r++){
    // 	for(int c = 0; c < groundImage.cols; c++){
    // 		// cout << "int the for looppppp";
    // 		processPixelRGB(c, r);
    // 	}
    // }
    // cout << "after the fooooor looooop";
}

void processPixelInRectangle(int r, int c, bool boundaryRight, bool boundaryLeft, int bound_right, int bound_left, int rect_top){
    RGB = comparisonImage.ptr<Point3_<uchar> >(r,c);
#ifdef DIV_BY_16
    // totalValue = ((RGB->z/16) * 10000) + ((RGB->y/16) * 100) + (RGB->x/16);
	if(GroundColors[RGB->z/16][RGB->y/16][RGB->x/16] > 0){
		outputImage.at<Vec3b>(Point(c,r)) = Vec3b(255,255,255);
		groundPix++;
	}
	else{
		outputImage.at<Vec3b>(Point(c,r)) = Vec3b(0,0,0);
		obstaclePix++;
	}

#elif (DIV_BY_8)
   // cout << "before AssignSegment...\n";
    int seg = AssignSegment(r, c, bound_right, bound_left);
   // cout << " seg = " << seg << endl;
    segments[seg].second++;
    // segments[c/(outputImage.cols/4)][(r-(outputImage.rows/2))/(outputImage.rows/8)].second++;
    // int s_x = getSegmentsX(c, outputImage);
    // int s_y = getSegmentsY(r, outputImage);
    // int s_x = c/(outputImage.cols/4);
    // int s_y = (r-(outputImage.rows/2))/(outputImage.rows/8);
    // int s_y = (8*r - 4*outputImage.rows)/outputImage.rows;
    // segments[s_x][s_y].second++;
    // cout << "div" << endl;
    if(GroundColors[RGB->z/8][RGB->y/8][RGB->x/8] > 0){
      outputImage.at<Vec3b>(Point(c,r)) = Vec3b(255,255,255);
        // cout << "white" << endl;
        groundPix++;
    }
    else{
        // segments[c/(outputImage.cols/4)][(r-(outputImage.rows/2))/(outputImage.rows/8)].first++;
        // segments[s_x][s_y].first++;
        segments[seg].first++;
     outputImage.at<Vec3b>(Point(c,r)) = Vec3b(0,0,0);
        // cout << "Black" << endl;
        CurrentFrameColors[RGB->z/8][RGB->y/8][RGB->x/8] = 1;
        obstaclePix++;

    }

    //Draw segments on frame
    // if((c % (comparisonImage.cols/4) == 0) || (r % (comparisonImage.rows/8) == 0)){
    //         outputImage.at<Vec3b>(Point(c,r)) = Vec3b(127, 127, 127);
    // }
    if(c == trapezoidBaseLeft_x || c == trapezoidBaseRight_x || boundaryRight || boundaryLeft || c == trapezoidMidpoint || r == rect_top){
     //  if( r == 295 )
   //     cout << " top rectangle at y = 295, x = " << c << endl;
     //   if( r == 340 )
     //       cout << " top rectangle at y = 340, x = " << c << endl;
     //   if( r == 400 )
       //     cout << " top rectangle at y = 400, x = " << c << endl;
//        if( r == rect_top)
//                cout << " r == rect_top == " << r << endl;

        outputImage.at<Vec3b>(Point(c,r)) = Vec3b(127, 127, 127);
    }
    // cout << "End of rectangle" << endl;
    // totalValue = ((RGB->z/8) * 10000) + ((RGB->y/8) * 100) + (RGB->x/8);
    // GroundColors[RGB->z/8][RGB->y/8][RGB->x/8]++;


#else
    if(GroundColors[RGB->z][RGB->y][RGB->x] > 0){
		outputImage.at<Vec3b>(Point(c,r)) = Vec3b(255,255,255);
		groundPix++;
	}
	else{
		outputImage.at<Vec3b>(Point(c,r)) = Vec3b(0,0,0);
		obstaclePix++;
	}

#endif

}

void processRectangle(Point top_left, Point top_right, Point bottom_left, Point bottom_right){
    float top_row = top_left.y;
    float bottom_row = bottom_left.y;

    float left_y = bottom_left.y - top_left.y;
    float left_x = bottom_left.x - top_left.x;

    float right_y = bottom_right.y- top_right.y;
    float right_x = bottom_right.x - top_right.x;

    float slope_right = right_y/right_x;
    float slope_left = left_y/left_x;


    // cout << "tr: " << top_row << " " << "br: " << bottom_row << endl;
    // cout << "sr: " << slope_right << " " << "sl: " << slope_left << endl;

    // slope_right = (slope_right < .8) ? .8 : slope_right;
    // slope_left = (slope_left < .8) ? .8 : slope_left;
    // slope_right *= 1.3;
    // slope_left = -slope_right;
    // int slope_right = 1;
    // int slope_left = 1;

//   trapezoid bounds
//    float boundary_left = top_left.x;
//    float boundary_right = top_right.x;

    //rect bounds
    float boundary_left = 260;
    float boundary_right = 379;
    float rect_top = 295;

    for(int r = trapezoidMaxHeight; r < comparisonImage.rows; r++) {
        // cout << "Processing row " << r << endl;
        for (int c = 0; c < comparisonImage.cols; c++) {
            // cout << c << endl;

            //last two parameters are used to draw gray lines on trapezoid diagonals

            // uncomment the next line for production code
            processPixelInRectangle(r, c, ((int) boundary_right == c), ((int) boundary_left == c), (int) boundary_right,
                                    (int) boundary_left, (int)rect_top);
        }

        if (r == 295) {
            boundary_left = 221; boundary_right = 418; rect_top = 340;
        //cout << " top rectangle at y = 295, x = " << c << endl;
         }
        if( r == 340 ) {
            boundary_left = 169; boundary_right = 470; rect_top = 400;
 //           cout << " rectop = " << rect_top << endl;
          //  cout << " top rectangle at y = 340, x = " << c << endl;
        }
        if( r == 400 ) {
            boundary_left = 100; boundary_right = 540; //rect_top = r;
            //cout << " top rectangle at y = 400, x = " << c << endl;
        }

//        boundary_left += 1/slope_left;
//        boundary_right += 1/slope_right;
    }
}


void compareImages(Point pt1, Point pt2, Point pt3, Point pt4){
    outputImage = comparisonImage.clone();
   // string window_name = "Temp Name";
    //make the output image the same size as the other two
    int compCols = comparisonImage.cols;
    int compRows = comparisonImage.rows;

//    cout << "Begin processing Rectangle" << endl;
    processRectangle(pt1, pt2, pt3, pt4);
//    cout << "Finish processing Rectangle" << endl;

    RGBOutputFrame = outputImage.clone();
    //Mat element = getStructuringElement( MORPH_RECT, Size(1,1));
    //erode(RGBOutputFrame, RGBOutputFrame, element);

    //string window_name = "test";
    //namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
    //imshow(window_name, HSVOutputImage);
#ifndef RUNTIME_SEQUENCE
    namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
	imshow(window_name, RGBoutputImage);
#endif
}

//start of candidate identification
void identifyPossibleGround(){

    int HistoryColors[maxDim+2][maxDim+2][maxDim+2] = {0};


    //copy values to HistoryColors
    for(int i = 0; i < maxDim; i++){
        for(int j = 0; j < maxDim; j++){
            for(int k = 0; k < maxDim; k++){
                HistoryColors[i + 1][j + 1][k + 1] = GroundColors[i][j][k];
                PossibleGroundColors[i][j][k] = GroundColors[i][j][k];
                //numOnes++;
            }//for k
        }//for j
    }//for i

//identify 2's
    for(int i = 1; i <= maxDim; i++){
        for(int j = 1; j <= maxDim; j++){
            for(int k = 1; k <= maxDim; k++){
                if( HistoryColors[i][j][k] == 0 ) {
                    if (HistoryColors[i + 1][j][k] == 1 ||
                        HistoryColors[i - 1][j][k] == 1 ||
                        HistoryColors[i][j + 1][k] == 1 ||
                        HistoryColors[i][j - 1][k] == 1 ||
                        HistoryColors[i][j][k + 1] == 1 ||
                        HistoryColors[i][j][k - 1] == 1)
                    {
                        HistoryColors[i][j][k] = 2;
                        PossibleGroundColors[i-1][j-1][k-1] = 2;
                        //cout << " HistoryColors[" << i << "][" << j << "][" << k << "] = " << HistoryColors[i][j][k] << endl;
                    } //end of if( any neighbors are == 1 )
                }
            }//for k
        }//for j
    }//for i

//identify 3's
    for(int i = 1; i <= maxDim; i++){
        for(int j = 1; j <= maxDim; j++){
            for(int k = 1; k <= maxDim; k++){
                if( HistoryColors[i][j][k] == 0 ) {
                    if (HistoryColors[i + 1][j][k] == 2 ||
                        HistoryColors[i - 1][j][k] == 2 ||
                        HistoryColors[i][j + 1][k] == 2 ||
                        HistoryColors[i][j - 1][k] == 2 ||
                        HistoryColors[i][j][k + 1] == 2 ||
                        HistoryColors[i][j][k - 1] == 2)
                    {
                        HistoryColors[i][j][k] = 3;
                        PossibleGroundColors[i-1][j-1][k-1] = 3;
                        //cout << " HistoryColors[" << i << "][" << j << "][" << k << "] = " << HistoryColors[i][j][k] << endl;
                    } //end of if( any neighbors are == 1 )
                }
            }//for k
        }//for j
    }//for i


}//end of identifyPossibleGround()

void updateGroundColors(){
    for(int i = 0; i < maxDim; i++){
        for(int j = 0; j < maxDim; j++){
            for(int k = 0; k < maxDim; k++) {
                if (CurrentFrameColors[i][j][k] == 1 && PossibleGroundColors[i][j][k] == 2) {
                    GroundColors[i][j][k] = 1;
                    PossibleGroundColors[i][j][k] = 1;
                }
                //numOnes++;
            }//for k
        }//for j
    }//for i
}

void loadImages(char** argv, int argc){
    /* Robot Init */
    /*
    if(init()==-1){
       exit(1);
    }
    led_on(1);
    */

    groundImage = imread(argv[1]);
    comparisonImage = imread(argv[2]);
    //VideoCapture cap(argv[2]);
    cap.open(argv[2]);
    if (!cap.isOpened()) //try to open and capture from a video file; if this fails, try to convert the arg to an int
        cap.open(atoi(argv[2]));//try to use an int version of the arg to connect to a camera
    if (!cap.isOpened()) {//test if camera connected
        cerr << "Failed to capture from " << argv[2] << " as an image, video file, and even as a camera!\n" << endl;
        exit(-1);//accept failure and die quickly
    }
}




string examineSegments(){
    bool leftRect =  SEGMENT_THRESHOLD < (segments[0].first/(float)segments[0].second);
    bool leftTri =  SEGMENT_THRESHOLD < (segments[1].first/(float)segments[1].second);
    bool leftTrap =  SEGMENT_THRESHOLD / 2 < (segments[2].first/(float)segments[2].second);
    bool rightTrap =  SEGMENT_THRESHOLD / 2 < (segments[3].first/(float)segments[3].second);
    bool rightTri =  SEGMENT_THRESHOLD < (segments[4].first/(float)segments[4].second);
    bool rightRect =  SEGMENT_THRESHOLD < (segments[5].first/(float)segments[5].second);

    if(leftTri && leftTrap){
        return "turn right";
    }
    else if(rightTrap && rightTri){
        return "turn left";
    }
    else if(leftTrap || rightTrap){
        return "stop";
    }
    else if(leftTri && rightTri){
        return "stop";
    }
}


VideoWriter ClearWrite("clear.avi", CV_FOURCC('M','J','P','G'), 25, frameSize,true);

    void runTimeAnalysisOfCurrentFrame(int propComms, VideoWriter &DmWrite, Point pt1, Point pt2, Point pt3, Point pt4, int &nextStep){
        groundPix = obstaclePix = 0;



            for(int i = 0; i < MAX_SEGMENTS; i++){
            segments[i].first = 0;//obstacles
            segments[i].second = 0;//total pixels in segment area
        }
        Mat flippedImage;
        flip(comparisonImage, flippedImage, 1);
        ClearWrite << flippedImage;
//    cout << "before compare images...\n";
        compareImages(pt1, pt2, pt3, pt4);
//cout << "after compare images...\n";


      //  u_char com_akt = pathVector.at(nextStep);

            float blackWhiteRatio = 0.0f;
            if (useNearFieldNav)
                blackWhiteRatio =
                        (float) (segments[11].first + segments[12].first) / (segments[11].second + segments[12].second);
            else //2,3,6,7,8,9
                //blackWhiteRatio = (float) obstaclePix/(obstaclePix+groundPix) ;
                blackWhiteRatio =
                        (float) (segments[2].first + segments[3].first + segments[6].first + segments[7].first +
                                 segments[8].first + segments[9].first) /
                        (segments[2].second + segments[3].second + segments[6].second + segments[7].second +
                         segments[8].second + segments[9].second);

            /* Stop and Go Condition */
            //    string printToFrame = "";
            if (blackWhiteRatio > .07) { // consider where we are and where we want to go
                digitalWrite(fwdPin, LOW);

            } else { // follow the path in this case
                digitalWrite(fwdPin, HIGH);
                if(blackWhiteRatio < .02)
                    updateGroundColors();


            }
            //} // end of else
            if (!DmWrite.isOpened()) //if VideoWriter is not initialized successfully, exit the program
            {
                cout << "ERROR: Failed to write the video" << endl;
                exit(-1);
            }
        flip(RGBOutputFrame, flippedImage, 1);

//        string printToFrame = "";
//        stringstream ss;
//        ss << "Step number : " << nextStep << " obstacleRatio: " << blackWhiteRatio << printToFrame;
//
//
//        putText(flippedImage, ss.str(), Point(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 255), 1, CV_AA);

        DmWrite << flippedImage;
}


struct hostent *server;
struct sockaddr_in serv_addr;

/********************************************************
 *
 * Create socket and bind if successful created
 *
 *********************************************************/
void bindSocket() {
    struct sockaddr_in serv_addr;       // structure containing server address

    /* enable keep-alive on the socket */
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    int idletime = 120; /* in seconds */
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idletime, sizeof(idletime));

    /* First call to socket() function */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        fprintf(stderr, "unable to create socket: %s\n", strerror(errno));
        exit(1);
    }

    /* fill in socket structure */
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    /* Now bind the host address using bind() call.*/
    rc = bind(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if (rc == -1) {
        fprintf(stderr, "unable to bind to socket: %s\n", strerror(errno));
        exit(1);
    }
}

bool setupServer(string cam) {

    /* create Internet domain socket */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        cout << "ERROR, no Connection established" << endl;
        return 0;
    }

    const char* cludgeTemp = cam.c_str();
    server = gethostbyname(cludgeTemp); // enter IP-address of Raspberry network

    if (server == NULL) {
        cout << "ERROR, no such server" << endl;
        return 0;
    }

    bzero((char *) &serv_addr, sizeof(serv_addr)); // sets all values of the client address to zero
    serv_addr.sin_family = AF_INET; // set address family
    serv_addr.sin_port = htons(portno); // set port number

    bcopy((char *) server->h_addr,  // copy new values into buffer
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);

    /* connect to server*/

    if (connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        cout << "ERROR connecting" << endl;
        return 0;
    } else
        cout << "Connection established " << endl;

    return 1;
}

/********************************************************
 *
 * write receiving data into text-file
 *
 *********************************************************/
void receiveData() {

    FILE *data = fopen(fr_name, "w"); // open txt file in read write mode rewrite the file every time

    char revbuf[buf_size]; // receive buffer for file
    if (data == NULL) {
        fprintf(stderr, "unable to open '%s': %s\n", data, strerror(errno));
        exit(1);
    } else {
        bzero(revbuf, buf_size);
        int fr_block_sz = 0;
        while ((fr_block_sz = recv(sock, revbuf, sizeof(revbuf), 0)) > 0) {
            int write_sz = fwrite(revbuf, sizeof(char), fr_block_sz, data);
            if (write_sz < fr_block_sz) {
                cout << "File write failed." << endl;
            }
            bzero(revbuf, buf_size);
            if (fr_block_sz == 0 || fr_block_sz != buf_size) {
                break;
            }
        }
        if (fr_block_sz < 0) {
            if (errno == EAGAIN) {
                printf("recv() timed out.\n");
            } else {
                fprintf(stderr, "recv() failed due to errno = %d\n", errno);
            }
        }
        cout << "Writing successfully done!" << endl;
        fclose(data);
    }
}


/********************************************************
 *
 * read data from text-file and write into vector
 *
 *********************************************************/
vector<u_char> readData() {
    char * line = NULL;
    size_t len = 0;
    FILE *path;
    vector<u_char> pathVector;

    path = fopen(fr_name, "r");
    if (path == NULL) {
        fprintf(stderr, "unable to open '%s': %s\n", path, strerror(errno));
        exit(1);
    }

    // read txt file line by line and write it into vector
    while ((getline(&line, &len, path)) != -1) {
        //read file into array
        pathVector.push_back(*line); //use vector for easier processing
    }

    fclose(path);
    cout << "close File" << endl;
    if (line)
        free(line);
    return pathVector;
}


void sendInt( int propComms, int value){
    char a[sizeof(int)];


    sprintf(a, "%d", value);
    for(int i = 0; i < sizeof(int); i++){
        serialPutchar(propComms, a[i]);
    }
}

int getInt(int propComms){
    char a[sizeof(int)];
    int returnValue;
    for(int i = 0; i < sizeof(int); i++){
        a[i] = serialGetchar(propComms);

    }
    sscanf(a, "%d", &returnValue);

    return returnValue;
}

int setPath(int propComms, vector<int> leftPath, vector<int> rightPath ){
    int numSteps = 0;

    //digitalWrite(1, LOW);
    digitalWrite(23, HIGH);
    digitalWrite(24, HIGH);

    //serialFlush(propComms);

    int catchKey = serialGetchar(propComms);

    cout << "char received indicating propellor serial ready : ";
    cout << (char) catchKey << endl;

    digitalWrite(23, LOW);
    digitalWrite(24, LOW);
    int i = 0;
    while(numSteps < leftPath.size() ){

        sendInt(propComms, leftPath.at(i));
        sendInt(propComms, rightPath.at(i));
        numSteps = getInt(propComms) ;
        cout << "Numsteps returned by propellor : ";
        cout << numSteps << endl;
        i++;

    }

    sendInt(propComms, 0);
    sendInt(propComms, 0);


    numSteps = getInt(propComms) ;
    numSteps = getInt(propComms) ;

//	int wtf = atoi(catchKey);
//	cin >> catchKey;
    cout << "Final Numsteps returned by propellor : ";
    cout << numSteps << endl;

    sendInt(propComms, 9);

    return numSteps;
}


void runPath(int propComms){

//
//    digitalWrite(1, HIGH);
//    int catchKey;
//    int leftTicks;
//    int rightTicks;
//
////wait until the propellor board reports path completion
//    while(cap.grab() && serialDataAvail(propComms) == 0){
//        cap.retrieve(comparisonImage);
////        cout << "inside while cap.read and nextStep less than vectorSize loop\n";
//        runTimeAnalysisOfCurrentFrame(propComms, DmWrite, pt1, pt2, pt3, pt4, nextStep);
//    }
//    digitalWrite(1, LOW);//stop movement
//    //make sure that the serial connection did not error out
//    if( serialDataAvail(propComms) == -1)
//        cout << " serial communcation error and errno = " << errno << endl;
//    else {
//        catchKey = getInt(propComms);
//        cout << "path completed " << catchKey << " instructions" << endl;
//
//    }

    digitalWrite(1, HIGH);
    int catchKey;
    int leftTicks;
    int rightTicks;

//wait until the propellor board reports path completion
    while(cap.grab() && serialDataAvail(propComms) == 0){
        cap.retrieve(comparisonImage);
//        cout << "inside while cap.read and nextStep less than vectorSize loop\n";
        runTimeAnalysisOfCurrentFrame(propComms, DmWrite, pt1, pt2, pt3, pt4, nextStep);
    }
    digitalWrite(1, LOW);//stop movement
    //make sure that the serial connection did not error out
    if( serialDataAvail(propComms) == -1)
        cout << " serial communcation error and errno = " << errno << endl;
    else {
        catchKey = getInt(propComms);
        cout << "path completed " << catchKey << " instructions" << endl;

    }



}

void closeSerial(int propComms){


    serialFlush(propComms);

//	printf("%c - %d", catchKey, catchKey);
    serialClose(propComms);

    cout << "closed serial \n";

}

int main(int argc, char ** argv){

    vector<int> leftPath;
    vector<int> rightPath;

    leftPath.push_back(1000);
    rightPath.push_back(1000);
//    leftPath.push_back(10);
//    rightPath.push_back(85);
//    leftPath.push_back(10);
//    rightPath.push_back(10);
//    leftPath.push_back(26);
//    rightPath.push_back(-25);

   /* Variables */
    maxR = maxG = maxB = 31;
    minR = minG = minB = 0;

    time_t start;
    time_t end;


    string videoFilename = "out.avi";
    VideoWriter DmWrite(videoFilename, CV_FOURCC('M','J','P','G'), 25, frameSize,true);

    loadImages(argv, argc); // LOAD IMAGES
//
//    Point pt1(320,265); //top left
//    Point pt2(378,265); //top right
//    Point pt3(80,480); //bottom left
//    Point pt4(520,480); //bottom right

    Point pt1(286,265); //top left
    Point pt2(354,265); //top right
    Point pt3(100,480); //bottom left
    Point pt4(540,480); //bottom right


    trapezoidTopLeft_x = pt1.x;
    trapezoidTopRight_x = pt2.x;
    trapezoidBaseLeft_x = pt3.x;
    trapezoidBaseRight_x = pt4.x;
    trapezoidMaxHeight = pt1.y;
    trapezoidMidpoint = ((pt2.x + pt1.x) / 2);
//set GPIO pins
    wiringPiSetup();
    pinMode(1, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);

    digitalWrite(1, LOW);
    digitalWrite(23, LOW);
    digitalWrite(24, LOW);

/* Ground Analysis */
    time(&start);
    for(int i = 0; i < 20;i++){
        cap.read(groundImage);
        processRectangleGround(pt1, pt2, pt3, pt4);
        identifyPossibleGround();
        processRectangle(pt1, pt2, pt3, pt4);
        updateGroundColors();
    }

//setup serial comms
    int propComms = serialOpen("/dev/ttyUSB0", 115200);

    if( propComms == -1 )
        cout << "Failed to open serial port. :( \n";



    serialFlush(propComms);

//setup multi cam parameters
    string cam[4];
    cam[0] = "";
    cam[1] = "192.168.1.146";
    cam[2] = "192.168.1.141";
    cam[3] = ""; //"192.168.1.144"; // currently on the ceiling

    string taskGoal = "";
    char detected[4] = {0};
    int nextCam = -1;
    int currentCam = -1;
    int goalCam = -1;
    bool wasDetected = false;
    int xGoal;
    int yGoal;

    bool isConnected = false;

//    cout << "Server Socket setup" << endl;
//
//
//

    cout << "Enter a destination ( Camera,X,Y,OriginArea ) with no spaces :";
    cin >> taskGoal;
    taskGoal += ","; // evil socket delimiter bull shit buffer overflow 2000 goals


    while( taskGoal != "-1" ){

        int j = 0;
        for( int i = 0; i < 4; i++){ // tokenize the taskGoal string
            string s;
            while(taskGoal[j] != ',' && j < strlen(taskGoal.c_str())) {
                s += taskGoal[j];
                j++;
            }
            cout << "S[" << i << "] = " << s << endl;
            j++; // discard the ','
            switch(i) {

                case 0:
                    goalCam = atoi(s.c_str());
                    break;
                case 1:
                    xGoal = atoi(s.c_str());
                    break;
                case 2:
                    yGoal = atoi(s.c_str());
                    break;
                case 3:
                    originArea = s[0];
                    break;
            }
        }

        // connect to the goal camera first, in case we are already in his region
        if( goalCam == 1){
            isConnected = setupServer(cam[goalCam]);
            currentCam = goalCam;
            nextCam = 2;
        }
        else{
            isConnected = setupServer(cam[goalCam]);
            currentCam = goalCam;
            nextCam = 1;
        }
/*
		while( !isConnected ){
			cout << "Failed to connect to cam1, attempting to connect to cam2" << endl;
			isConnected = setupServer(cam2);
			currentCam = 2;
			if( !isConnected ) {
				cout << "Failed to connect to cam2, attempting to connect to cam1" << endl;
				isConnected = setupServer(cam1);
				currentCam = 1;
			}

		}
*/
        send(sock, taskGoal.c_str(), strlen(taskGoal.c_str()), 0); // send a msg to goalCam to see if it can see me

        int recv_sz = 0;
        recv_sz = recv(sock, detected, sizeof(detected), 0); // receive the response from goalCam
        j = 0;
        for( int i = 0; i < 2; i++){
            string s;
            while(detected[j] != ',' && j < 3) {
                s += detected[j];
                if (detected[j] == '-')
                    s+= detected[j+1];
                j++;
            }
            cout << "S[" << i << "] = " << s << endl;
            j++; // discard the ','
            switch(i) {

                case 0:
                    wasDetected = s[0] == 'T';
                    break;
                case 1:
                    if (wasDetected)
                        nextCam = atoi(s.c_str());
                    break;
            }
        }
        cout << "nextCam: " << nextCam << endl;

        bool taskComplete = false;

        while( !taskComplete){

            if( !wasDetected ){ // if the goal camera didn't detect me
                close(sock);

                isConnected = setupServer(cam[nextCam]);
                currentCam = nextCam;
                nextCam = goalCam;
                send(sock, taskGoal.c_str(), strlen(taskGoal.c_str()), 0); // send taskGoal to other cam

                recv_sz = recv(sock, detected, sizeof(detected), 0); // receive his response

                j = 0;
                for( int i = 0; i < 2; i++){ // tokenize
                    string s;
                    while(detected[j] != ',' && j < 3) {
                        s += detected[j];
                        if (detected[j] == '-')
                            s+= detected[j+1];
                        j++;
                    }
                    cout << "S[i] = " << s << endl;
                    j++; // discard the ','
                    switch(i) {

                        case 0:
                            wasDetected = s[0] == 'T';
                            break;
                        case 1:
                            if (wasDetected)
                                nextCam = atoi(s.c_str());
                            break;
                    }
                }
                cout << "was not detected and nextCam: " << nextCam << endl;

            }
            else {	// if goal camera detected me

                //Start of copy paste for test


                const char* imReady = "T";
                cout << "I'm asking ceiling camera for path" << endl;
                send(sock, imReady, strlen(imReady), 0); // ask for the path

                int recv_sz = recv(sock, pathLength, sizeof(pathLength), 0); // receive the response from goalCam


                cout << "bad alloc chaser\n";

                const int pathSize = atoi(pathLength);
                cout << "pathSize = " << pathSize << endl;
                if (pathSize == 0) {
                    wasDetected = false;
                    cout << "Camera could not find a path.  Aborting task.\n";
                } else {
                    char receivedPath[pathSize];
                    recv_sz = recv(sock, receivedPath, pathSize, 0); // receive the response from goalCam


                    cout << "receivedPath = " << receivedPath << endl;

                    cout << "printing each char\n";

                    for (int i = 0; i < pathSize; i++) {

                        cout << receivedPath[i];

                    }
                    cout << endl;
//                receiveData();
                    vector<int> path;
                    path.clear();
                    leftPath.clear();
                    rightPath.clear();


//                path = readData();
                    cout << "bad alloc chaser\n";

                    int numTokens = 0;
                    for (int i = 0; i < pathSize; i++) {
                        if (receivedPath[i] == ',')
                            numTokens++;
                    }

                    cout << "numTokens : " << numTokens << endl;

                    int j = 0;
                    for (int i = 0; i < numTokens; i++) { // tokenize
                        string st;
                        while (receivedPath[j] != ',') {
                            st += receivedPath[j];
                            j++;
                        }
                        cout << "S[i] = " << st << endl;
                        j++; // discard the ','
                        path.push_back(atoi(st.c_str()));
                    }

                    cout << " Received path : ";
                    for (int i = 0; i < path.size(); i += 2) {
                        cout << path.at(i) << endl;
                        cout << path.at(i + 1) << endl;
                        leftPath.push_back(path.at(i));
                        rightPath.push_back(path.at(i + 1));
                    }

                    int numSteps = setPath(propComms, leftPath, rightPath);
                    runPath(propComms);



                    //end of copy and paste for test


                    if (nextCam == -1)
                        taskComplete = true;
                    else {
                        //char currentCamChar;
                        //itoa(currentCam, currentCamChar, 10);
                        taskGoal[taskGoal.length() - 2] = '0' + currentCam;
                        cout << " taskGoal updated and now is : " << taskGoal << endl;
                    }

                    wasDetected = false;
                    cout << "was detected and nextCam: " << nextCam << endl;
                }
            }


        }// end of while ( currentCam != goalCam)
        close(sock);

        cout << "Enter a destination ( Camera,X,Y ) with no spaces :";
        cin >> taskGoal;
        taskGoal += ","; // evil socket delimiter bull shit buffer overflow 2000 goals
    }//End of while( taskGoal != "-1" ) loop




//    int numSteps = setPath(propComms, leftPath, rightPath);
//  //  runPath(propComms);
////contents of runPath are dumped below for debugging:
//
//    digitalWrite(1, HIGH);
//    int catchKey;
//    int leftTicks;
//    int rightTicks;
//
////wait until the propellor board reports path completion
//    while(cap.grab() && serialDataAvail(propComms) == 0){
//        cap.retrieve(comparisonImage);
////        cout << "inside while cap.read and nextStep less than vectorSize loop\n";
//        runTimeAnalysisOfCurrentFrame(propComms, DmWrite, pt1, pt2, pt3, pt4, nextStep);
//    }
//    digitalWrite(1, LOW);//stop movement
//    //make sure that the serial connection did not error out
//    if( serialDataAvail(propComms) == -1)
//        cout << " serial communcation error and errno = " << errno << endl;
//    else {
//        catchKey = getInt(propComms);
//        cout << "path completed " << catchKey << " instructions" << endl;
//
//    }
////end of runPath dump
//


    closeSerial(propComms);


//
    return 0;
}
