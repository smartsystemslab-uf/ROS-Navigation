#include <iostream>
#include <vector>

//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;
/*
class coordinate{
public:
	int p;
	int min;
	int max;

	coordinate() {
		p = 0;
		min = 0;
		max = 0;
	}


};
*/

class TransitionRegion {
public:


    int erosion_type = 0;
    int erosion_size = 1;
    Mat element = getStructuringElement( erosion_type,
                                         cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
    
	int xPoint;
	int yPoint;
	int xMin;
	int yMin;
	int xMax;
	int yMax;

    Mat testBinaryImage;

    enum TRDirection {ENTERING, EXITING};

    TransitionRegion();
    TransitionRegion( int xmin, int ymin, int xmax, int ymax);

    void setPoint(int xp, int yp);
    void setImage(string filename);
    bool checkBorderingPixels(int x, int y, Mat img);

    static vector<TransitionRegion> getTopTransitionRegions(Mat &image);
    static vector<TransitionRegion> getBottomTransitionRegions(Mat &image);
    static vector<TransitionRegion> getLeftTransitionRegions(Mat &image);
    static vector<TransitionRegion> getRightTransitionRegions(Mat &image);


    void detectRobotTop(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &topBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y);
    void detectRobotBottom(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &topBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y);
    void detectRobotLeft(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &topBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y);
    void detectRobotRight(vector<int> &x_locations, vector<int> &y_locations, const vector<TransitionRegion> &topBorder, const Mat &clearGroundImage, const Mat &robotImage, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y);

    void getRobotLocationsLeftRight(vector<int> &x_locations, vector<int> &y_locations, const Mat &roiDiff, const TransitionRegion border, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y);
    void getRobotLocationsTopBottom(vector<int> &x_locations, vector<int> &y_locations, const Mat &roiDiff, const TransitionRegion border, vector<int> &robot_sizes_x,vector<int> &robot_sizes_y);


    void print();
    TransitionRegion& operator=(const TransitionRegion& other);
    const TransitionRegion& operator+(const TransitionRegion& other);
    

};
