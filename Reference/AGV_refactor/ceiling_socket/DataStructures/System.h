#include "../Imaging/Aruco.h"
#include "../RobotControl/Planner.h"



/*  */
class Robot {
private:
    string robID; // robot's hostname. "rob1", "rob2"
    int robType; // idk?
    int robSizeX; // max dimension X of robot
    int robSizeY; // max dimension Y of robot
    float robTickDistCM; // tick distance of robot in centimeters
    string prevLocation; // previous camera or zone from grid
    string goalLocation; // goal camera/zone
    int finalGoalX; // coordinates in GOAL ceiling
    int finalGoalY; // coordinates in GOAL ceiling
    int goalX; // goal coordinates in my area
    int goalY; // goal coordinates in my area
    string path; // initial path that is sent to bot
    int numCorrections; // # of corrections made
    // vector of turn coord
    // vector of obstacle coord


    enum STATUS{ROBOT_FOUND, ROBOT_NOT_FOUND, ROBOT_BROKEN, ROBOT_STUCK};
public:

    Robot() {

    };

    Robot(string id) {
      robID = id;
    };

    ~Robot() {

    };
};


/*  */
class Ceiling {

private:
    int camID; // neighbor id
    string camIP;
    int gridRow;
    int gridCol;

    vector<Ceiling> neighbors; // 0 north, 1 south, 2 west, 3 east
    vector<Robot> robots;
    vector<Aruco> arcuoDetectors;
    vector<TransitionRegion> leftTransitionRegions;
    vector<TransitionRegion> rightTransitionRegions;
    vector<TransitionRegion> topTransitionRegions;
    vector<TransitionRegion> bottomTransitionRegions;


public:

    Ceiling(){
        planner = new Planner();
    };

    ~Ceiling(){
        delete planner;
    }

    Ceiling(int cID, string cIP, int row, int col){
        camID = cID;
        camIP = cIP;
        gridRow = row;
        gridCol = col;
    }
    Planner *planner;
//

    Mat* originalImage;
    Mat* binaryImage;
    Mat* runningImage;


//  remove this


    int getCamID() const { return camID; }
    const string &getCamIP() const { return camIP; }
    int getGridRow() const { return gridRow; }
    int getGridCol() const { return gridCol; }
    const vector<Ceiling> &getNeighbors() const { return neighbors; }
    const vector<Robot> &getRobots() const { return robots; }
    const vector<Aruco> &getArcuoDetectors() const { return arcuoDetectors; }
    const vector<TransitionRegion> &getLeftTransitionRegions() const { return leftTransitionRegions; }
    const vector<TransitionRegion> &getRightTransitionRegions() const { return rightTransitionRegions; }
    const vector<TransitionRegion> &getTopTransitionRegions() const { return topTransitionRegions; }
    const vector<TransitionRegion> &getBottomTransitionRegions() const { return bottomTransitionRegions; }

    void setCamID(int camID) { Ceiling::camID = camID; }
    void setCamIP(const string &camIP) { Ceiling::camIP = camIP; }
    void setGridRow(int gridRow) { Ceiling::gridRow = gridRow; }
    void setGridCol(int gridCol) { Ceiling::gridCol = gridCol; }
    void setNeighbors(const vector<Ceiling> &neighbors) { Ceiling::neighbors = neighbors; }
    void setRobots(const vector<Robot> &robots) { Ceiling::robots = robots; }
    void setArcuoDetectors(const vector<Aruco> &arcuoDetectors) { Ceiling::arcuoDetectors = arcuoDetectors; }
    void setLeftTransitionRegions(const vector<TransitionRegion> &leftTransitionRegions) { Ceiling::leftTransitionRegions = leftTransitionRegions; }
    void setRightTransitionRegions(const vector<TransitionRegion> &rightTransitionRegions) { Ceiling::rightTransitionRegions = rightTransitionRegions; }
    void setTopTransitionRegions(const vector<TransitionRegion> &topTransitionRegions) { Ceiling::topTransitionRegions = topTransitionRegions; }
    void setBottomTransitionRegions(const vector<TransitionRegion> &bottomTransitionRegions) { Ceiling::bottomTransitionRegions = bottomTransitionRegions; }
// transition regions

  	// needs a set func that merges transition regions
    //forward_list<Robot> robots; /* debug code FIXME */
};
