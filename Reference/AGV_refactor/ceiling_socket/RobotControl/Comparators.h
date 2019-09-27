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

