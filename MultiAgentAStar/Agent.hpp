#pragma once

#include <vector>

typedef std::vector<unsigned int> RouteVector;

class Agent {
public:
	Agent(void);
	Agent(unsigned int _x, unsigned int _y);

	void setStart(unsigned int _x, unsigned int _y);
	void setRoute(RouteVector _route);
	void addRouteStep(unsigned int direction);

	unsigned int getStartX(void);
	unsigned int getStartY(void);
	unsigned int getID(void);
	std::vector<unsigned int> getRoute(void);
	void deleteRoute();

private:
	unsigned int mID;
	unsigned int StartX, StartY;
	RouteVector route;
	
	static unsigned int ID;
};
