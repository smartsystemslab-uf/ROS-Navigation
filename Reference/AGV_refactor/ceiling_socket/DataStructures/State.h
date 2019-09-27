#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <vector>
#include <stack>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

class State {

public:

	virtual inline void findNeighbors(std::vector<State>& neighbors, const int bufferDistance) {}

    virtual inline bool operator<(const State& other) const {}

    virtual inline bool operator==(const State& other) const {}

    friend ostream &operator<<(ostream &output, const State &S) {}

};

#endif // STATE_H
