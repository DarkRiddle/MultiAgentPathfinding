#pragma once
#include "DistancePolicy.hpp"

// Definition Details
template <typename DistancePolicy>
class Node_ : private DistancePolicy {
	using DistancePolicy::calculateDistance;
public:
	Node_(int xp, int yp, int lvl, int prio) : xPos(xp), yPos(yp), level(lvl), priority(prio){};

	// Basic getters
	int getxPos(void) const { return xPos; }
	int getyPos(void) const { return yPos; }
	int getLevel(void) const { return level; }
	int getPriority(void) const { return priority; }

	void updatePriority(const int& xDst, const int& yDst);
	void nextLevel(const int& i);

private:
	int xPos, yPos;
	int level, priority;
};


// Implementation Details
template <typename DP>
void Node_<DP>::updatePriority(const int& xDst, const int& yDst) {
	priority=level+calculateDistance(xPos,yPos,xDst,yDst)*10;
}

template <typename DP>
void Node_<DP>::nextLevel(const int& i) {
	level+=(i%2==0?10:14);
}

template <typename DP>
bool operator<(const Node_<DP>& a, const Node_<DP>& b)
{
	return a.getPriority() > b.getPriority();
}

typedef Node_<DistanceEuclidian> EuclidianNode;
typedef Node_<DistanceManhattan> ManhattanNode;
