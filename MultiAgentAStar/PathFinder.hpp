#pragma once

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <array>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include "Agent.hpp"
#include "Node.hpp"
typedef EuclidianNode Node;

class PathFinder {
public:
	PathFinder(int n, int m);
	~PathFinder(void) {};
	void AStar();
	void printPath();
	void setStart(int a_x, int a_y, int b_x, int b_y);
	void setTarget(int x, int y);
	bool Test(int x, int y);
	void Find();
	void setAgent(Agent* A);
	void addAgent(Agent* A);
	unsigned int getTargetX();
	unsigned int getTargetY();
	void clearListAgents();
	void initMap(void);
	
private:
	int map_w;
	int map_h;
	int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
	int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
	
	int AxStart, AyStart;
	int BxStart, ByStart;
	int xObj,yObj;
	int startX, startY;
	int robotID;
	
	int** map;
	int** closed_nodes_map;
	int** open_nodes_map;
	int** dir_map;
	int** old_map;
	
	std::vector<Agent*> listAgents;
	std::vector<unsigned int> route;
	
	int** testMap;
	int** oldMap;
	std::vector<unsigned int> testRoute;
	std::vector<unsigned int> savedRoute;
};
