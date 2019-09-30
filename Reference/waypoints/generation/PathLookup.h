/*

*/
#include <iostream>
#include <vector>
#include <string>

class PathLookup {
  public:
    int start_x, start_y;
    int goal_x, goal_y;
    int robotSize;
    string pathString;

    PathLookup() { }
    PathLookup(int s_x, int s_y, int g_x, int g_y, int rS, string pS) {
      start_x = s_x;
      start_y = s_y;
      goal_x = g_x;
      goal_y = g_y;
      robotSize = rS;
      pathString = pS;
    }
    ~PathLookup() { }

    friend ostream &operator<<( ostream &output, const PathLookup &obj ) {
  		output << "(" << obj.start_x << "," << obj.start_y << ") to (" << obj.goal_x << "," << obj.goal_y << ") for ";
      output << "robot size of " << obj.robotSize << " with \npath: " << obj.pathString << endl;
  		return output;
  	}

    friend ofstream &operator<<( ofstream &output, const PathLookup &obj ) {
  		output << obj.start_x << " " << obj.start_y << " " << obj.goal_x << " " << obj.goal_y << " ";
      output << obj.robotSize << " " << obj.pathString << endl;
  		return output;
  	}

    /* does not work - static errors and shit. */
    /*
    friend ifstream &operator>>( ifstream &input, const PathLookup &obj ) {
  		input >> obj.start_x >> obj.start_y >> obj.goal_x >> obj.goal_y >> obj.robotSize;
  		return input;
  	}
    */
};
