/*
* PathPlanning.h
*
*  Created on: Apr 8, 2016
*      Author: Streit FJ
*      email: StreitFr50552@th-nuernberg.de
*/

#include "Planner.h"
#include "Comparators.h"

int Planner::regionCMX = -1;
int Planner::regionCMY = -1;
int Planner::goal_x = -1;
int Planner::goal_y = -1;
double Planner::bestCost = 99999;
int Planner::imgCount = 0;

double Planner::calcTicksPerPixel(float tickDistance, float robotCM, float robotPixels) {
    return ((robotCM/robotPixels)/tickDistance);
}

void Planner::calcRegionArea(float robotCM, float robotPixels, Mat region) {
    regionCMX = (int)(robotCM/robotPixels) * region.cols;
    regionCMY = (int)(robotCM/robotPixels) * region.rows;
}

void Planner::rebuildPath(stack<PixelState> &pathStack, const int gridSize, Mat &binaryImage, const string &srcCam) {
    //cout << "rebuilding path which has size " << pathStack.size() << endl;
  		PixelState *last = &pathStack.top(); // goal state
      PixelState *current = last->parent; // goal state's parent
      PixelState *next = current->parent; // goal state's parent's parent

      binaryImage.at<Vec3b>(Point(last->x, last->y)) = Vec3b(60,255,60); // drawing goalState
      binaryImage.at<Vec3b>(Point(last->x-1, last->y)) = Vec3b(60,255,60); // drawing goalState
      binaryImage.at<Vec3b>(Point(last->x+1, last->y)) = Vec3b(60,255,60); // drawing goalState
      binaryImage.at<Vec3b>(Point(last->x, last->y-1)) = Vec3b(60,255,60); // drawing goalState
      binaryImage.at<Vec3b>(Point(last->x, last->y+1)) = Vec3b(60,255,60); // drawing goalState

  		while (next != NULL) {
  			//cout << "we're at step: " << aux->x << "," << aux->y << endl;
        if ((current->x == last->x && current->y == last->y)) {
          cout << "basically ignore this. last and current were same point" << endl;
        }
        else if ((last->x == current->x && current->x != next->x)
                || (last->y == current->y && current->y != next->y)
                || ((last->x == current->x || last->y == current->y) && (current->x != next->x && current->y != next->y))
                || (last->x != current->x && last->y != current->y && (current->y == next->y || current->x == next->x))
                || (last->x != current->x && last->y != current->y && next->y != current->y && next->x != current->x && (next->x == last->x || next->y == last->y)))
        {
          pathStack.push(*current);
    			//cout << "pushed that aux" << endl;
          current->drawCircles(gridSize, binaryImage);
        }
        else {
          binaryImage.at<Vec3b>(Point(current->x, current->y)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x+1, current->y)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x-1, current->y)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x, current->y+1)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x, current->y-1)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x+1, current->y+1)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x-1, current->y+1)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x+1, current->y-1)) = Vec3b(255,20,255);
          binaryImage.at<Vec3b>(Point(current->x-1, current->y-1)) = Vec3b(255,20,255);
        }

        last = current;
  			current = next;
        next = next->parent;
  		}

      binaryImage.at<Vec3b>(Point(current->x, current->y)) = Vec3b(60,255,60); // drawing startState
      binaryImage.at<Vec3b>(Point(current->x-1, current->y)) = Vec3b(60,255,60); // drawing startState
      binaryImage.at<Vec3b>(Point(current->x+1, current->y)) = Vec3b(60,255,60); // drawing startState
      binaryImage.at<Vec3b>(Point(current->x, current->y-1)) = Vec3b(60,255,60); // drawing startState
      binaryImage.at<Vec3b>(Point(current->x, current->y+1)) = Vec3b(60,255,60); // drawing startState
		  //cout << "path was rebuilt" << endl;
}

int Planner::determineStartOrientation(int x, int y) {
    if (x < 50) {
        return 2;
    } else if (x > 600) {
        return 6;
    } else if (y < 80) {
        return 4;
    } else if (y > 410) {
        return 0;
    } else if (x < y) {
        return 2;
    } else return 6;
}

PixelState Planner::AStar(PixelState startState, PixelState goalState, string &path, int bufferDistance, Mat &binaryImage, const string &srcCam, const string &nextCam, int alg, int robSize, double ticksPerPixel, double &costResult) {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    std::multiset<PixelState, AStarComparator> frontier;
    std::set<PixelState, StateComparator> beenthere;
    std::vector<PixelState> neighbors;
    int gridSize = 8;

    //cout << "Goal has been previously set: " << startState.goal_x << ", " << startState.goal_y << endl;
    goal_x = startState.goal_x;
    goal_y = startState.goal_y;

    startState.cost = 0.0;
    startState.heuristic = startState.estCost(alg);
    startState.parent = NULL;

    frontier.insert(startState);
    beenthere.insert(startState);

    PixelState *s;

    int xx = 0;

    // debug spew
    /*Mat tmp;
    while (xx == 0) {
      PixelState pixel(startState);
      cout << "input col: ";
      cin >> xx;
      pixel.x = xx;
      cout << "input row: ";
      cin >> xx;
      pixel.y = xx;
      bool tooClose = false;
      cout << "map.rows: " << binaryImage.rows << "  map.cols: " << binaryImage.cols << "  bufferDistance: " << bufferDistance << endl;
      int colMin = 999;
      int rowMin = 999;
      int colMax = -1;
      int rowMax = -1;
      int badPixCol = -1;
      int badPixRow = -1;
      for (int i = -bufferDistance; i <= bufferDistance; i++) {
        cout << "row " << pixel.y + i << ": ";
        for (int j = -bufferDistance; j <= bufferDistance; j++) {
          if (rowMin > pixel.y + i)
            rowMin = pixel.y + i;
          if (colMin > pixel.x + j)
            colMin = pixel.x + j;
          if (rowMax < pixel.y + i)
            rowMax = pixel.y + i;
          if (colMax < pixel.x + j)
            colMax = pixel.x + j;
          if ((binaryImage.rows > pixel.y + i) && (pixel.y + i >= 0) && (binaryImage.cols > pixel.x + j) && (pixel.x + j >= 0)) {
            Vec3b color = binaryImage.at<Vec3b>(Point(pixel.x + j, pixel.y + i));
            if (j == -bufferDistance)
              cout << color << " ... ";
            else if (color != Vec3b(255,255,255))
              cout << color << " ... ";
            if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
              cout << "\nPixel we're checking: " << pixel.x << "," << pixel.y << " is too close to " << pixel.x + j << "," << pixel.y + i << " with color " << color << endl;
              tooClose = true;
              badPixCol = pixel.x + j;
              badPixRow = pixel.y + i;
              break;
            }
          }
          if (tooClose)
            break;
        }
        if (tooClose)
          break;
        cout << endl;
      }
      if (tooClose) {
        cout << "buffer is claiming that this pixel is too close to an obstacle!" << endl;
      }
      else {
        cout << "valid pixel to add" << endl;
      }
      Mat tmp = binaryImage.clone();
      circle(tmp, Point(pixel.x, pixel.y), 4, CV_RGB(255, 0, 0), 1, 8);
      rectangle(tmp, Point(colMin, rowMin), Point(colMax, rowMax), CV_RGB(0, 255, 0), 1, 8, 0);
      if (tooClose)
        tmp.at<Vec3b>(Point(badPixCol, badPixRow)) = Vec3b(0,0,255);
      imshow("window", tmp);
      waitKey(0);
      destroyAllWindows();
      cout << "loop done: " << endl;
      cin >> xx;
    }
    imshow("window2", binaryImage);
    waitKey(0);
    destroyAllWindows();
    */
    // debug spew

    while (frontier.size() > 0) {
      //end = std::chrono::high_resolution_clock::now();
      //time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

      s = new PixelState;
      (*s) = (*frontier.begin());

      if ((abs(s->x - goalState.x) < gridSize) && (abs(s->y - goalState.y) < gridSize)) {
        cout << "Path cost: " << s->cost << endl;
        cout << "Path cost h: " << s->heuristic << endl;
        if (s->cost < bestCost) {
          cout << "This path " << imgCount << " is cheaper than our current best path: prev " << bestCost << " > new " << s->cost << endl;
          costResult = s->cost;
          bestCost = costResult;
          goalState.parent = s;
          pathStack.push(goalState); // push goal state
          rebuildPath(pathStack, gridSize, binaryImage, srcCam);
          path = writePathFile(startState, gridSize, srcCam, nextCam, robSize, ticksPerPixel);
          cout << "  path is: " << path << endl;
          char filename[200];
          sprintf(filename,"pathOnBinary%.3d.jpg",imgCount++);
          imwrite(filename, binaryImage);
          return (*s);
        }
        else {
          cout << "This path " << imgCount << " is not cheaper: prev " << bestCost << " > new " << s->cost << endl;
          costResult = s->cost;
          goalState.parent = s;
          pathStack.push(goalState); // push goal state
          rebuildPath(pathStack, gridSize, binaryImage, srcCam);
          path = writePathFile(startState, gridSize, srcCam, nextCam, robSize, ticksPerPixel);
          cout << "  path is: " << path << endl;
          char filename[200];
          sprintf(filename,"pathOnBinary%.3d.jpg",imgCount++);
          imwrite(filename, binaryImage);
          return startState;
        }
      }

      frontier.erase(frontier.begin());
      PixelState *oldChild = new PixelState;
      /*
      xx++; // debug
      if (xx > 50) { // debug
        cin >> xx; // debug
        cout << "Printing frontier: " << endl;
        for (std::set<PixelState>::iterator it = frontier.begin(); it != frontier.end(); it++) {
          cout << "  " << *it << endl;
        }
        cin >> xx;
        cout << "Printing beenthere: " << endl;
        for (std::set<PixelState>::iterator it = beenthere.begin(); it != beenthere.end(); it++) {
          cout << "  " << *it << endl;
        }
        cin >> xx;
      } // debug
      */
      //cout << "Looking for neighbors for point " << s << " at " << s->x << "," << s->y << endl;
      s->findNeighbors(neighbors, gridSize, bufferDistance, binaryImage, alg);

      for (unsigned int i = 0; i < neighbors.size(); i++) {
        //cout << "\nNeighbor i " << i << ": (" << neighbors[i].x << ", " << neighbors[i].y << ") with c: " << neighbors[i].cost << " and h: " << neighbors[i].heuristic << endl;
        PixelState *child = new PixelState;
        (*child) = neighbors[i];
        //cout << "child: (" << (*child).x << ", " << (*child).y << ") with c: " << (*child).cost << " and h: " << (*child).heuristic << endl;
        double actionCost = neighbors[i].cost;
        //cout << "action cost: " << actionCost << endl;

        bool childFound = false; // unfortunate manual set::find because I messed up comparator above somehow
        for (std::set<PixelState>::iterator it = beenthere.begin(); it != beenthere.end(); it++) {
          if ((*it).x == (*child).x && (*it).y == (*child).y) {
            childFound = true;
            break;
          }
        }

        if (childFound) {
          //cout << "Child was found in beenthere" << endl;

          std::set<PixelState>::iterator it; // again, unfortunate manual set::find because I messed up comparator above somehow
          for (it = beenthere.begin(); it != beenthere.end(); it++) {
            if ((*it).x == (*child).x && (*it).y == (*child).y) {
              break;
            }
          }

          if (it != beenthere.end()) {
            (*oldChild) = (*it);
            //cout << "oldChild: (" << (*oldChild).x << ", " << (*oldChild).y << ") with c: " << (*oldChild).cost << " and h: " << (*oldChild).heuristic << endl;

            if (s->cost + actionCost < (*oldChild).cost) {
              //cout << "new child is cheaper. replacing old stats:" << endl;
              (*oldChild).cost = s->cost + actionCost;
              (*oldChild).parent = s;
              //cout << "oldChild: (" << (*oldChild).x << ", " << (*oldChild).y << ") with c: " << (*oldChild).cost << " and h: " << (*oldChild).heuristic << endl;
            }
          }
          else {
            cout << "equal_range did not find another child with same value in beenthere" << endl;
          }
        }
        else {
          //cout << "Child was not found in beenthere. never been visited. add it" << endl;
          (*child).cost = s->cost + actionCost;
          (*child).parent = s;
          //cout << "child: (" << (*child).x << ", " << (*child).y << ") with c: " << (*child).cost << " and h: " << (*child).heuristic << endl;
          frontier.insert((*child));
          beenthere.insert((*child));
        }
      }
    }
    cout << "Couldn't find path to the goal." << endl;
    return startState;
}

PixelState Planner::UCS(PixelState startState, PixelState goalState, string &path, int bufferDistance, Mat &binaryImage, const string &srcCam, const string &nextCam,int robSize, double ticksPerPixel) {
   std::multiset<PixelState > pqueue;
        std::set<PixelState> closed_set;
        typename std::set<PixelState>::iterator set_it;
        std::vector<PixelState > neighbors;
        int gridSize = 4;

        startState.cost = 0.0;
        startState.parent = NULL;
        pqueue.insert(startState);
        closed_set.insert(startState);

        pops = 0;
        its = 0;
        PixelState *s;

        while (pqueue.size() > 0) {
            s = new PixelState;
            (*s) = (*pqueue.begin());
            pqueue.erase(pqueue.begin());
            pops++;

            bool closeEnough = false;
            for(int xCount = -((gridSize/2)+2); xCount < gridSize+2; xCount++) {
                for(int yCount = -((gridSize/2)+2); yCount < gridSize+2; yCount++) {
                    if((s->x + xCount == goalState.x) && (s->y + yCount == goalState.y)) {
                        closeEnough = true;
                    }
                }
            }

            if (closeEnough) {
              goalState.parent = s;
              pathStack.push(goalState); // push goal state
              rebuildPath(pathStack, gridSize, binaryImage, "srcCam");
                path = writePathFile(startState, gridSize, srcCam, nextCam, robSize, ticksPerPixel);
                return (*s);
            }

            s->findNeighbors(neighbors, gridSize, bufferDistance, binaryImage, 0);
            for (size_t i = 0; i < neighbors.size(); i++) {
                neighbors[i].parent = s;
                neighbors[i].cost = s->cost
                                      + neighbors[i].determineCost(*s);

                set_it = closed_set.find(neighbors[i]);
                if (set_it != closed_set.end()) {
                    if (neighbors[i].heuristic < (*set_it).heuristic) {
                        std::pair<multisetIt_t, multisetIt_t> ret =
                                pqueue.equal_range((*set_it));
                        if (ret.first != pqueue.end()) {
                            for (multisetIt_t pqit = ret.first; pqit != ret.second; pqit++)
                                if ((*pqit) == (*set_it)) {
                                    pqueue.erase(pqit);
                                    pqueue.insert(neighbors[i]);
                                    break;
                                }
                        }
                        closed_set.erase(set_it);
                        closed_set.insert(neighbors[i]);
                    }
                } else {
                    pqueue.insert(neighbors[i]);
                    closed_set.insert(neighbors[i]);
                }
            }

            if (its % 5000 < 1000)
                s->recordProgress();

            its++;
        }
        cout << "Coudn't find path to the goal" << endl;
        return startState;
}

PixelState Planner::BFS(PixelState startState, PixelState goalState, string &path, int bufferDistance, Mat &binaryImage, const string &srcCam, const string &nextCam,int robSize, double ticksPerPixel) {
      //cout << "in bfs" << endl;
        std::queue<PixelState> pqueue;
        std::set<PixelState, StateComparator> closed_set;
        typename std::set<PixelState>::iterator cset_it;
        std::vector<PixelState> neighbors;
        int gridSize = 8;

        startState.cost = 0.0;
        startState.parent = NULL;
        pqueue.push(startState);
        closed_set.insert(startState);
      	//cout << "pushed startState." << endl;
      	//cout << "Goal has been previously set: " << startState.goal_x << ", " << startState.goal_y << endl;
        pops = 0;
        its = 0;
        PixelState *s;

        while (pqueue.size() > 0) {
            s = new PixelState;
            (*s) = pqueue.front();
            pqueue.pop();
            pops++;

            if ((abs(s->x - goalState.x) < gridSize) && (abs(s->y - goalState.y) < gridSize)) {
                goalState.parent = s;
                pathStack.push(goalState); // push goal state
                rebuildPath(pathStack, gridSize, binaryImage, srcCam);
                path = writePathFile(startState, gridSize, srcCam, nextCam, robSize, ticksPerPixel);
                return (*s);
            }

	          //cout << "Looking for neighbors for point " << s << " at " << s->x << "," << s->y << endl;
            s->findNeighbors(neighbors, gridSize, bufferDistance, binaryImage, 0);

            //cout << "We found " << neighbors.size() << " neighbors." << endl;
            for (size_t i = 0; i < neighbors.size(); i++) {
                cset_it = closed_set.find(neighbors[i]);
                if (cset_it != closed_set.end())
                    continue;

		            //cout << "  pre: neighbor " << i << " has parent of " << neighbors[i].parent << endl;
                neighbors[i].parent = s;
		            //cout << "  set: neighbor " << i << " has parent of " << neighbors[i].parent << endl;
                neighbors[i].cost = s->cost + neighbors[i].determineCost(*s);
                closed_set.insert(neighbors[i]);
                pqueue.push(neighbors[i]);
            }

            if (its % 5000 < 1000)
                s->recordProgress();

            its++;
        }

        cout << "Coudn't find path to the goal" << endl;
        return startState;
}

string Planner::writePathFile(PixelState startState, int gridSize, const string &srcCam, const string &nextCam, int robSize, double ticksPerPixel)  {

    pathFile.open("path.txt");
    PixelState *s;
    PixelState *prev;
    string path;
    int prev_x = startState.x;
    int prev_y = startState.y;
    int curr_x = -1;
    int curr_y = -1;
    int diff_x = -1;
    int diff_y = -1;
    int prevOrientation = -1;
    cout << "source cam: " << srcCam << endl;
    switch(atoi(srcCam.c_str())){
        case 0:
            prevOrientation = 4;
            break;
        case 1:
            prevOrientation = 6;
            break;
        case 2:
            prevOrientation = 0;
            break;
        case 3:
            prevOrientation = 2;
            break;
        default:
            cout << "Houston, we have a problem. srcCam = " << srcCam << endl;
            prevOrientation = 0;
            return "";
            break;
    }
    cout << "Initial orientation: " << prevOrientation << endl;
    //pathStack.pop();
    while(pathStack.size() > 0){
        s = &pathStack.top();
        pathStack.pop();
        curr_x = s->x;
        curr_y = s->y;
        diff_x = curr_x - prev_x;
        diff_y = curr_y - prev_y;
        int leftMotorTurn = 0;
        int rightMotorTurn = 0;
        int motorsForward = 0;

        //determine turn angle
        if(diff_x != 0 && diff_y != 0){
            double ratio = abs(diff_x/diff_y);
            //45 degree turn first as it should be most common
            if(.754 < ratio && ratio <= 1.33){
                switch (prevOrientation) {
                    case 0:
                        if (diff_x > 0) {
                            // 45 right
                            leftMotorTurn = 12;
                            rightMotorTurn = -12;
                            prevOrientation = 1;
                        }
                        else {
                            // 45 left
                            leftMotorTurn = -12;
                            rightMotorTurn = 12;
                            prevOrientation = 7;
                        }
                        break;
                    case 2:
                        if (diff_y > 0) {
                            // 45 right
                            leftMotorTurn = 12;
                            rightMotorTurn = -12;
                            prevOrientation = 3;
                        }
                        else {
                            // 45 left
                            leftMotorTurn = -12;
                            rightMotorTurn = 12;
                            prevOrientation = 1;
                        }
                        break;
                    case 4:
                        if (diff_x < 0) {
                            // 45 right
                            leftMotorTurn = 12;
                            rightMotorTurn = -12;
                            prevOrientation = 5;
                        }
                        else {
                            // 45 left
                            leftMotorTurn = -12;
                            rightMotorTurn = 12;
                            prevOrientation = 3;
                        }
                        break;
                    case 6:
                        if (diff_y < 0) {
                            // 45 right
                            leftMotorTurn = 12;
                            rightMotorTurn = -12;
                            prevOrientation = 7;
                        }
                        else {
                            // 45 left
                            leftMotorTurn = -12;
                            rightMotorTurn = 12;
                            prevOrientation = 5;
                        }
                        break;
                }
            }
            else if(.134 < ratio && ratio <= .424){
                //15
                cout << "15 degree turn. not coded." << endl;
                cout << "  from: (" << prev_x << "," << prev_y << ") to (" << curr_x << "," << curr_y << ")" << endl;
            }
            else if(.425 < ratio && ratio <= .754){
                //30
                cout << "30 degree turn. not coded." << endl;
                cout << "  from: (" << prev_x << "," << prev_y << ") to (" << curr_x << "," << curr_y << ")" << endl;
            }
            else if(1.34 < ratio && ratio <= 2.35){
                //60
                cout << "60 degree turn. not coded." << endl;
                cout << "  from: (" << prev_x << "," << prev_y << ") to (" << curr_x << "," << curr_y << ")" << endl;
            }
            else if(2.36 < ratio && ratio <= 8.14){
                //75
                cout << "75 degree turn. not coded." << endl;
                cout << "  from: (" << prev_x << "," << prev_y << ") to (" << curr_x << "," << curr_y << ")" << endl;
            }
            else if( 8.15 < ratio){
                //90 from diagonal to diagonal
                cout << "prevOr: " << prevOrientation << endl;
                switch (prevOrientation) {
                    case 1:
                        if (diff_x > 0) {
                            // turn right 90
                            leftMotorTurn = 25;
                            rightMotorTurn = -25;
                            prevOrientation = 3;
                        }
                        else if (diff_x < 0) {
                            // turn left 90
                            leftMotorTurn = -25;
                            rightMotorTurn = 25;
                            prevOrientation = 7;
                        }
                        break;
                    case 3:
                        if (diff_y > 0) {
                            // turn right 90
                            leftMotorTurn = 25;
                            rightMotorTurn = -25;
                            prevOrientation = 5;
                        }
                        else if (diff_y < 0) {
                            // turn left 90
                            leftMotorTurn = -25;
                            rightMotorTurn = 25;
                            prevOrientation = 1;
                        }
                        break;
                    case 5:
                        if (diff_x < 0) {
                            // turn right 90
                            leftMotorTurn = 25;
                            rightMotorTurn = -25;
                            prevOrientation = 7;
                        }
                        else if (diff_x > 0) {
                            // turn left 90
                            leftMotorTurn = -25;
                            rightMotorTurn = 25;
                            prevOrientation = 3;
                        }
                        break;
                    case 7:
                        if (diff_y < 0) {
                            // turn right 90
                            leftMotorTurn = 25;
                            rightMotorTurn = -25;
                            prevOrientation = 1;
                        }
                        else if (diff_y > 0) {
                            // turn left 90
                            leftMotorTurn = -25;
                            rightMotorTurn = 25;
                            prevOrientation = 5;
                        }
                        break;
                }
                cout << "leftmotor: " << leftMotorTurn << " rightmotor: " << rightMotorTurn << endl;
            }
            else {
                //0 (straight along diagonal)
                cout << "This shouldn't happen. 0 degree turn (going straight along a diagonal)" << endl;
                cout << "  from: (" << prev_x << "," << prev_y << ") to (" << curr_x << "," << curr_y << ")" << endl;
            }
        }
        else{
            // either no turn or 90 degree turn
            switch (prevOrientation) {
                case 0:
                    if (diff_x > 0) {
                        // turn right 90
                        leftMotorTurn = 25;
                        rightMotorTurn = -25;
                        prevOrientation = 2;
                    }
                    else if (diff_x < 0) {
                        // turn left 90
                        leftMotorTurn = -25;
                        rightMotorTurn = 25;
                        prevOrientation = 6;
                    }
                    break;
                case 1:
                    if (diff_y == 0) {
                        // 45 right
                        leftMotorTurn = 12;
                        rightMotorTurn = -12;
                        prevOrientation = 2;
                    }
                    else if (diff_x == 0 && diff_y < 0) {
                        // 45 left
                        leftMotorTurn = -12;
                        rightMotorTurn = 12;
                        prevOrientation = 0;
                    }
                    else if (diff_x == 0 && diff_y > 0) {
                        // 135 right
                        cout << "Turn right 135 degrees. Fix path finding Dillon, damn it." << endl;
                        cout << "  previous orientation was 1." << endl;
                        return "";
                    }
                    break;
                case 2:
                    if (diff_y > 0) {
                        // turn right 90
                        leftMotorTurn = 25;
                        rightMotorTurn = -25;
                        prevOrientation = 4;
                    }
                    else if (diff_y < 0) {
                        // turn left 90
                        leftMotorTurn = -25;
                        rightMotorTurn = 25;
                        prevOrientation = 0;
                    }
                    break;
                case 3:
                    if (diff_y == 0) {
                        // 45 left
                        leftMotorTurn = -12;
                        rightMotorTurn = 12;
                        prevOrientation = 2;
                    }
                    else if (diff_x == 0 && diff_y > 0) {
                        // 45 right
                        leftMotorTurn = 12;
                        rightMotorTurn = -12;
                        prevOrientation = 4;
                    }
                    else if (diff_x == 0 && diff_y < 0) {
                        // 135 left
                        cout << "Turn left 135 degrees. Fix path finding Dillon, damn it." << endl;
                        cout << "  previous orientation was 3." << endl;
                        return "";
                    }
                    break;
                case 4:
                    if (diff_x < 0) {
                        // turn right 90
                        leftMotorTurn = 25;
                        rightMotorTurn = -25;
                        prevOrientation = 6;
                    }
                    else if (diff_x > 0) {
                        // turn left 90
                        leftMotorTurn = -25;
                        rightMotorTurn = 25;
                        prevOrientation = 2;
                    }
                    break;
                case 5:
                    if (diff_x == 0) {
                        // 45 left
                        leftMotorTurn = -12;
                        rightMotorTurn = 12;
                        prevOrientation = 4;
                    }
                    else if (diff_y == 0 && diff_x < 0) {
                        // 45 right
                        leftMotorTurn = 12;
                        rightMotorTurn = -12;
                        prevOrientation = 6;
                    }
                    else if (diff_y == 0 && diff_x > 0) {
                        // 135 left
                        cout << "Turn left 135 degrees. Fix path finding Dillon, damn it." << endl;
                        cout << "  previous orientation was 5." << endl;
                        return "";
                    }
                    break;
                case 6:
                    if (diff_y < 0) {
                        // turn right 90
                        leftMotorTurn = 25;
                        rightMotorTurn = -25;
                        prevOrientation = 0;
                    }
                    else if (diff_y > 0) {
                        // turn left 90
                        leftMotorTurn = -25;
                        rightMotorTurn = 25;
                        prevOrientation = 4;
                    }
                    break;
                case 7:
                    if (diff_y == 0) {
                        // 45 left
                        leftMotorTurn = -12;
                        rightMotorTurn = 12;
                        prevOrientation = 6;
                    }
                    else if (diff_x == 0 && diff_y < 0) {
                        // 45 right
                        leftMotorTurn = 12;
                        rightMotorTurn = -12;
                        prevOrientation = 0;
                    }
                    else if (diff_x == 0 && diff_y > 0) {
                        // 135 left
                        cout << "Turn left 135 degrees. Fix path finding Dillon, damn it." << endl;
                        cout << "  previous orientation was 7." << endl;
                        return "";
                    }
                    break;
            }
        }

        //set tick count
	       //cout << "ticksperPixel: " << ticksPerPixel << endl;
        int ticksNeeded = std::sqrt((diff_x * diff_x) + (diff_y * diff_y)) * ticksPerPixel;
	       //cout << "ticksNeeded: " << ticksNeeded << endl;
        motorsForward = ticksNeeded;

        // append to path file and string
        if (leftMotorTurn != 0 && rightMotorTurn != 0) {
            pathFile << "(" << to_string(leftMotorTurn) << ",";
            pathFile << to_string(rightMotorTurn) << ")" << endl;
            path += to_string(leftMotorTurn) + "," + to_string(rightMotorTurn) + ",";
        }
	if (motorsForward != 0) {
		pathFile << "(" << to_string(motorsForward) << ",";
		pathFile << to_string(motorsForward) << ")" << endl;
		path += to_string(motorsForward) + "," + to_string(motorsForward) + ",";
		pathFile << "(1,1)" << endl;
		path += "1,1,";
	}

        // update points
        prev_x = curr_x;
        prev_y = curr_y;
    }
    int leftTurnTicks = 0;
    int rightTurnTicks = 0;
    //add pivot
    if(nextCam == "0" && prevOrientation != 0){
        //effective orientation = 0
        if(prevOrientation > 4){
            leftTurnTicks = 12*(prevOrientation-4);
            rightTurnTicks = -leftTurnTicks;
        }
        else{
            rightTurnTicks = 12*prevOrientation;
            leftTurnTicks = -rightTurnTicks;
        }
    }
    else if(nextCam == "1" && prevOrientation != 2){
        //effective orientation = 2
        int correction = prevOrientation - 2;
        if(correction < 0) correction += 8; //pO = 0 -> correction = -2 -> correction = 6
        if(correction > 4){
            leftTurnTicks = 12*(correction-4);
            rightTurnTicks = -leftTurnTicks;
        }
        else{
            rightTurnTicks = 12*correction;
            leftTurnTicks = -rightTurnTicks;
        }
    }
    else if(nextCam == "2" && prevOrientation != 4){
        //effective orientation = 4
        int correction = prevOrientation - 4;
        if(correction < 0) correction += 8; //pO = 0 -> correction = -2 -> correction = 6
        if(correction > 4){
            leftTurnTicks = 12*(correction-4);
            rightTurnTicks = -leftTurnTicks;
        }
        else{
            rightTurnTicks = 12*correction;
            leftTurnTicks = -rightTurnTicks;
        }
    }
    else if(nextCam == "3" && prevOrientation != 6){
        //effective orientation = 6
        int correction = prevOrientation - 6;
        if(correction < 0) correction += 8; //pO = 0 -> correction = -2 -> correction = 6
        if(correction > 4){
            leftTurnTicks = 12*(correction-4);
            rightTurnTicks = -leftTurnTicks;
        }
        else{
            rightTurnTicks = 12*correction;
            leftTurnTicks = -rightTurnTicks;
        }

    }
    if(rightTurnTicks != 0 && leftTurnTicks != 0){
        pathFile << "(" << to_string(leftTurnTicks) << ",";
        pathFile << to_string(rightTurnTicks) << ")" << endl;
        path += to_string(leftTurnTicks) + "," + to_string(rightTurnTicks) + ",";
    }
    return path;
}

bool Planner::checkValidPoints(const Mat &img, int &x, int &y, int bufferDistance) {
    if (x < 20)
        x = 20;
    if (y < 20)
        y = 20;
    if (x > 620)
        x = 620;
    if (y > 460)
        y = 460;
    if (img.at<Vec3b>(Point(x, y)) == Vec3b(0,0,0) || img.at<Vec3b>(Point(x, y)) == Vec3b(1,1,1))
        return false;
    for (int i = -bufferDistance/2; i <= bufferDistance/2; i++) {
        for (int j = -bufferDistance/2; j <= bufferDistance/2; j++) {
            if ((img.rows > y + i) && (y + i >= 0) && (img.cols > x + j) && (x + j >= 0)) {
                Vec3b color = img.at<Vec3b>(Point(x + j, y + i));
                if (color.val[0] < 32 && color.val[1] < 32 && color.val[2] < 32) {
                    return false;
                }
            }
        }
    }
    return true;
}


string Planner::doPointPathFinding(int startXpath, int startYpath, int goalXpath, int goalYpath, int alg) {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    //openClose.release(); // was getting lots of errors with this


//    binaryImage = imread("subtractedImage.jpg");
//    bufferDistance = ((sqrt(pow(robotSizesX[0], 2) + pow(robotSizesY[0], 2))) / 2) + 2;
//    if (!(checkValidPoints(startXpath, startYpath, goalXpath, goalYpath))) {
//        cout << "You cannot start or end on an obstacle" << endl;
//        //circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
//        //circle(openClose, Point(goalXpath, goalYpath), 4, CV_RGB(0, 0, 255), 3, 8);
//        //rectangle(openClose, Point(startXpath-30, startYpath-30), Point(startXpath+30, startYpath+30), CV_RGB(0,255,0), 1, 8);
//        //rectangle(openClose, Point(goalXpath-30, goalYpath-30), Point(goalXpath+30, goalYpath+30), CV_RGB(0,255,0), 1, 8);
//        //imshow("test", openClose);
//        //waitKey(0);
//        return "";
//    }
//
//    PixelState::goal_x = goalXpath;
//    PixelState::goal_y = goalYpath;
//    string p = "";
//    bool foundSavedPath = false;
//
//    cout << "Detected a robot of size " << robotSizesX[0] << " by " << robotSizesY[0] << endl;
//    ticksPerPixel = calcTicksPerPixel(ACTIVITY_BOT_tickDistance_CM, ACTIVITY_BOT_SIZE_CM, std::max(robotSizesX[0], robotSizesY[0]));
//
//    // for testing w/o communication
//    /*if (startXpath < 50) {
//        srcCam = grid[myGridRow][myGridCol + 1]; // 2
//    } else if (startXpath > 600) {
//        srcCam = grid[myGridRow][myGridCol - 1]; // 6
//    } else if (startYpath < 80) {
//        srcCam = grid[myGridRow + 1][myGridCol]; // 4
//    } else if (startYpath > 410) {
//        srcCam = grid[myGridRow - 1][myGridCol]; // 0
//    } else if (startXpath < startYpath) {
//        srcCam = grid[myGridRow][myGridCol + 1]; // 2
//    } else srcCam = grid[myGridRow][myGridCol - 1]; // 6
//    cout << "srcCam detected as " << srcCam << endl; */
//    // end testing w/o communication
//    // once communication is set up, remove this FIXME
//
//    // change srcCam from the starting region letter ("E") into the neighbor side (0-3) in string form
//    for (unsigned int i = 0; i < neighbors.size(); i++) {
//        if (grid[neighbors[i].gridRow][neighbors[i].gridCol] == srcCam) {
//            srcCam = to_string(neighbors[i].side);
//            break;
//        }
//        else {
//            //cout << "debug:   grid[ ] is " << grid[neighbors[i].gridRow][neighbors[i].gridCol] << "  and srcCam is " << srcCam << endl;
//        }
//    }
//    cout << "srcCam overwritten to neighbor.side: " << srcCam << endl;
//
//    cout << "Attempting to find a path from (" << startXpath << "," << startYpath << ") to (";
//    cout << goalXpath << "," << goalYpath << ") with a buffer of " << bufferDistance << endl;
//
//    // check if start and goal have been calculated previously FIXME needs updating
//    if (savedPaths.size() > 0) {
//        for (int i = 0; i < savedPaths.size(); i++) {
//            if (savedPaths[i].start_x == startXpath && savedPaths[i].start_y == startYpath &&
//                savedPaths[i].goal_x == goalXpath && savedPaths[i].goal_y == goalYpath) {
//                foundSavedPath = true;
//                p = savedPaths[i].pathString;
//                break;
//            }
//        }
//    }
//
//    if (!foundSavedPath) { // if we cannot find a previously calculated path, run path finding
//        if (alg == 0) {
//            pathResult = BFS(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), path, bufferDistance, binaryImage, srcCam, nextCam, std::max(robotSizesX[0], robotSizesY[0]), ticksPerPixel);
//        }
//        else {
//            pathResult = AStar(PixelState(startXpath, startYpath), PixelState(goalXpath, goalYpath), path, bufferDistance, binaryImage, srcCam, nextCam, alg, std::max(robotSizesX[0], robotSizesY[0]), ticksPerPixel, cost);
//        }
//
//        if (pathResult == PixelState(startXpath, startYpath)) { // if no path exists
//            //circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
//            //circle(openClose, Point(goalXpath, goalYpath), 8, CV_RGB(255, 0, 0), 3, 8);
//            /*
//            openClose.at<Vec3b>(Point(startXpath, startYpath)) = Vec3b(60,255,60); // drawing startState
//              openClose.at<Vec3b>(Point(startXpath-1, startYpath)) = Vec3b(60,255,60); // drawing startState
//              openClose.at<Vec3b>(Point(startXpath+1, startYpath)) = Vec3b(60,255,60); // drawing startState
//              openClose.at<Vec3b>(Point(startXpath, startYpath-1)) = Vec3b(60,255,60); // drawing startState
//              openClose.at<Vec3b>(Point(startXpath, startYpath+1)) = Vec3b(60,255,60); // drawing startState
//              openClose.at<Vec3b>(Point(goalXpath, goalYpath)) = Vec3b(60,255,60); // drawing goalState
//              openClose.at<Vec3b>(Point(goalXpath-1, goalYpath)) = Vec3b(60,255,60); // drawing goalState
//              openClose.at<Vec3b>(Point(goalXpath+1, goalYpath)) = Vec3b(60,255,60); // drawing goalState
//              openClose.at<Vec3b>(Point(goalXpath, goalYpath-1)) = Vec3b(60,255,60); // drawing goalState
//              openClose.at<Vec3b>(Point(goalXpath, goalYpath+1)) = Vec3b(60,255,60); // drawing goalState
//            imwrite("pathOnBinary.jpg", openClose);
//            */
//            return p;
//        }
//        else { // else, a new path was found
//            PathLookup newPath(startXpath, startYpath, goalXpath, goalYpath, 999/*robotSize*/, path); // temporary object for pushing back new paths to vector
//            savedPaths.push_back(newPath);
//            p = path;
//            //circle(openClose, Point(startXpath, startYpath), 4, CV_RGB(255, 0, 0), 3, 8);
//            //circle(openClose, Point(goalXpath, goalYpath), 8, CV_RGB(255, 0, 0), 3, 8);
//            //imwrite("pathOnBinary.jpg", openClose);
//        }
//    }
//    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
//
//    return p;
}


