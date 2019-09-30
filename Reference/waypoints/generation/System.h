/*  */
class Robot {
  public:
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
  public:
    int camID; // neighbor id
    string camIP;
    int gridRow;
    int gridCol;
    int side; // 0 north, 1 south, 2 west, 3 east
    // transition regions

  	// needs a set func that merges transition regions
    //forward_list<Robot> robots; /* debug code FIXME */
};
