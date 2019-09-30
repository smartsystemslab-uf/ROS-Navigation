// compares strictly (x,y) of two PixelStates
struct StateComparator {
  bool operator() (const PixelState a, const PixelState b) const {
    if (a.y < b.y)
			return true;
		else if (a.y > b.y)
			return false;
		else if (a.x < b.x)
			return true;
		else
			return false;
    /*
    int aDist;
    int bDist;

    if (a.x - goal_x == 0 || a.y - goal_y == 0) {
      aDist = abs(a.x - goal_x) + abs(a.y - goal_y);
    }
    else {
      aDist = sqrt((pow((a.x - goal_x), 2) + pow((a.y - goal_y), 2)));
    }

    if (b.x - goal_x == 0 || b.y - goal_y == 0) {
      bDist = abs(b.x - goal_x) + abs(b.y - goal_y);
    }
    else {
      bDist = sqrt((pow((b.x - goal_x), 2) + pow((b.y - goal_y), 2)));
    }

    return aDist < bDist;
    */
  }
};

// compares the heuristic
struct AStarComparator {
  bool operator() (const PixelState a, const PixelState b) const {
    return (a.cost + a.heuristic) < (b.cost + b.heuristic);
  }
};

// compares the cost
struct UCSComparator {
    bool operator() (const PixelState a, const PixelState b) const {
      return a.cost < b.cost;
    }
};

