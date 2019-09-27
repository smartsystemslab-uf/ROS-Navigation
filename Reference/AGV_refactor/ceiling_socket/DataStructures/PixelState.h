/*
 * PixelState.h
 *
 *  Created on: Apr 19, 2016
 *      Author: Streit FJ
 *      email: StreitFr50552@th-nuernberg.de
 */
#ifndef PIXEL_STATE_H
#define PIXEL_STATE_H

#include <iostream>
#include <vector>
#include <stack>
#include <fstream>
#include <cmath>

#include "State.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

using namespace cv;
using namespace std;

class PixelState : State {
public:
	int x; // col
	int y; // row

	static int goal_x;
	static int goal_y;

  double cost;
  double heuristic;
  PixelState *parent;


	PixelState() : x(0), y(0), parent(NULL), cost(0), heuristic(0.0) { }
	PixelState(int x, int y) : x(x), y(y), parent(NULL), cost(0), heuristic(0.0) { }
	PixelState(const PixelState& in) : x(in.x), y(in.y), parent(in.parent), cost(in.cost), heuristic(in.heuristic) { }
	~PixelState() { }

//	bool isCloseEnough(PixelState goal, const int gridSize) {
//		if ((old_goal_x != goal_x) && (old_goal_y != goal_y)) {
//			old_goal_x = goal_x;
//			old_goal_y = goal_y;
//			goal_x = goal.x;
//			goal_y = goal.y;
//		}
//		for(int xCount = -((gridSize/2)+2); xCount < gridSize+2; xCount++) {
//			for(int yCount = -((gridSize/2)+2); yCount < gridSize+2; yCount++) {
//				if((x + xCount == goal.x) && (y + yCount == goal.y)) {
//					return true;
//				}
//			}
//		}
//		return false;
//	}

  // calculates the heuristic value of this pixel (future distance to goal)
	double estCost(int alg) {
		return sqrt((pow((x - goal_x), 2) + pow((y - goal_y), 2))); // euclidean heuristic
	}

	// finds all valid neighbors
	void findNeighbors(std::vector<PixelState>& neighbors, const int gridSize, const int bufferDistance, const Mat &map, int alg) {
			PixelState c;
			PixelState pi; // this PixelState's parent so we can tell if we turned
			if (parent == NULL) {
				//cout << "Finding neighbors for startState" << endl;
				pi.x = x;
				pi.y = y;
			}
			else {
				pi.x = parent->x;
				pi.y = parent->y;
			}
			neighbors.resize(0);
			Vec3b color = map.at<Vec3b>(Point(x, y));

			if(color.val[0] >= 32 && color.val[1] >= 32 && color.val[2] >= 32) {
				// cardinal directions
				if ((x < map.cols-gridSize)) {
					c.x = x + gridSize;
					c.y = y;
					c.cost = gridSize;
					if (((alg == 2)) && (pi.x != x && pi.y != y) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x+gridSize, y));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, gridSize, 0, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
				if ((x >= gridSize)) {
					c.x = x - gridSize;
					c.y = y;
					c.cost = gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x-gridSize, y));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, -gridSize, 0, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
				if ((y >= gridSize)) {
					c.x = x;
					c.y = y - gridSize;
					c.cost = gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x, y-gridSize));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, 0, -gridSize, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
				if ((y < map.rows-gridSize)) {
					c.x = x;
					c.y = y + gridSize;
					c.cost = gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x, y+gridSize));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, 0, gridSize, bufferDistance, map))
							neighbors.push_back(c);
					}
				}

				/* diagonal directions */
				if ((x < map.cols-gridSize) && (y < map.rows-gridSize)) {
					c.x = x + gridSize;
					c.y = y + gridSize;
					c.cost = 1.41 * gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x+gridSize, y));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, gridSize, gridSize, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
				if ((x < map.cols-gridSize) && (y >= gridSize)) {
					c.x = x + gridSize;
					c.y = y - gridSize;
					c.cost = 1.41 * gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x-gridSize, y));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, gridSize, -gridSize, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
				if ((x >= gridSize) && (y >= gridSize)) {
					c.x = x - gridSize;
					c.y = y - gridSize;
					c.cost = 1.41 * gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x, y-gridSize));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, -gridSize, -gridSize, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
				if ((x >= gridSize) && (y < map.rows-gridSize)) {
					c.x = x - gridSize;
					c.y = y + gridSize;
					c.cost = 1.41 * gridSize;
					if (((alg == 2)) && ((pi.x == x && c.x != x) || (pi.y == y && c.y != y) || (pi.x != x && pi.y != y && (c.y == y || c.x == x)))) {
						c.cost = c.cost + 2; // small punishment on turns. gridSize is only 4 so +2 is plenty but it needed to be more expensive to turn than go diagonal.
					}
					else if (pi.x != x && pi.y != y && c.y != y && c.x != x && (c.x == pi.x || c.y == pi.y)) {
						c.cost = c.cost + 4; // prevent turns that bounce between the diagonals (like /\/\/\/)
					}
					c.heuristic = c.estCost(alg);
					Vec3b colour = map.at<Vec3b>(Point(x, y+gridSize));
					if(colour.val[0] >= 32 && colour.val[1] >= 32 && colour.val[2] >= 32) {
						if(!isAroundObstacle(c, -gridSize, gridSize, bufferDistance, map))
							neighbors.push_back(c);
					}
				}
			}
		}

	// function to draw path pixels into the image
	void drawCircles(const int gridSize, Mat &map) {
		if (gridSize < 8) {
			map.at<Vec3b>(Point(x, y)) = Vec3b(255,20,20);
		}
		else {
    		map.at<Vec3b>(Point(x, y)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x+1, y)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x, y+1)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x-1, y)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x, y-1)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x+1, y+1)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x-1, y-1)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x+1, y-1)) = Vec3b(255,20,20);
			map.at<Vec3b>(Point(x-1, y+1)) = Vec3b(255,20,20);
		}
	}

	// function to draw the waves into the image FIXME this is unused. can modify it into something useful
	void recordProgress() {
		// does nothing
	}

	// calculates the cost of this pixel (distance from startState)
	double determineCost(const PixelState &src) {
		return (abs(x - src.x) + abs(y - src.y));
	}

	// returns true if the pixel we want to path toward is too close to a black pixel
	bool isAroundObstacle(const PixelState& pixel, int xChange, int yChange, const int bufferDistance, const Mat &map) {
		bool tooCloseToObstacle = false;

		for (int i = -bufferDistance; i <= bufferDistance; i++) { // rows, y
			for (int j = -bufferDistance; j <= bufferDistance; j++) { // cols, x
				if ((map.rows > pixel.y + i) && (pixel.y + i >= 0) && (map.cols > pixel.x + j) && (pixel.x + j >= 0)) {
					Vec3b color = map.at<Vec3b>(Point(pixel.x + j, pixel.y + i));
					if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
						//cout << "Pixel we're checking: " << pixel.x << "," << pixel.y << " is too close to " << pixel.x + i << "," << pixel.y + j << " with color " << color << endl; // debug
						return true;
						break;
					}
				}
			}
		}
		return false; // fuck this shit. redundancy is best

		if (xChange != 0 && yChange != 0) { // we are going diagonal
			for (int i = -bufferDistance; i <= bufferDistance; i++) { // rows, y
				for (int j = -bufferDistance; j <= bufferDistance; j++) { // cols, x
					if ((map.rows > pixel.y + i) && (pixel.y + i >= 0) && (map.cols > pixel.x + j) && (pixel.x + j >= 0)) {
						Vec3b color = map.at<Vec3b>(Point(pixel.x + j, pixel.y + i));
						if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
							//cout << "Pixel we're checking: " << pixel.x << "," << pixel.y << " is too close to " << pixel.x + i << "," << pixel.y + j << " with color " << color << endl; // debug
							tooCloseToObstacle = true;
							break;
						}
					}
				}
			}
		}
		if (tooCloseToObstacle) {
			return tooCloseToObstacle;
		}
		else {
			if (xChange != 0) { // we are changing columns
				for (int i = -bufferDistance; i <= bufferDistance; i++) {
					if ((map.cols > pixel.x + i) && (pixel.x + i >= 0)) { // bounds check
						//cout << "valid pixel at " << pixel.x + bufferDistance*PosNegToggle << "," << pixel.y + i << endl;
						Vec3b color = map.at<Vec3b>(Point(pixel.x, pixel.y + i));
						if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
							//cout << "Pixel we're checking: " << pixel.x << "," << pixel.y << " is too close to " << pixel.x + i << "," << pixel.y << " with color " << color << endl;
							tooCloseToObstacle = true;
							break;
						}
					}
				}
			}
			//cout << "x changes are done: " << tooCloseToObstacle << endl;
			if (tooCloseToObstacle) {
				return tooCloseToObstacle;
			}
			else {
				if (yChange != 0) { // we are changing rows
					for (int i = -bufferDistance; i <= bufferDistance; i++) {
						if ((map.rows > pixel.y + i) && (pixel.y + i >= 0)) { // bounds check
							Vec3b color = map.at<Vec3b>(Point(pixel.x + i, pixel.y));
							if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
								//cout << "Pixel we're checking: " << pixel.x << "," << pixel.y << " is too close to " << pixel.x + i << "," << pixel.y << " with color " << color << endl;
								tooCloseToObstacle = true;
								break;
							}
						}
					}
				}
			}
		}
		//cout << "returning a " << tooCloseToObstacle << endl;
		//return tooCloseToObstacle;
	}

	//
	inline bool operator==(const PixelState& other) const {
		return (x==other.x && y==other.y);
	}

	//
	inline bool operator=(const PixelState& other) {
		x = other.x;
		y = other.y;
		cost = other.cost;
		heuristic = other.heuristic;
    parent = other.parent;
		return true;
	}

	//
	inline bool operator<(const PixelState& other) const {
		if (y < other.y)
			return true;
		else if (y > other.y)
			return false;
		else if (x < other.x)
			return true;
		else
			return false;
	}

	//
	friend ostream &operator<<( ostream &output, const PixelState &S ) {
		output << "(" << S.x << "," << S.y << ") with cost: " << S.cost << " and heuristic: " << S.heuristic << ".";
		return output;
	}
};

#endif // PIXEL_STATE_H
