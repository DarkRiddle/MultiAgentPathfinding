#include "Agent.hpp"

unsigned int Agent::ID = 0;

Agent::Agent(void) : mID(ID++) {
	route = std::vector<unsigned int>();
}

Agent::Agent(unsigned int _x, unsigned int _y) : StartX(_x), StartY(_y), mID(ID++) {
	route = std::vector<unsigned int>();
}

void Agent::setStart(unsigned int _x, unsigned int _y) {
	StartX = _x;
	StartY = _y;
}

void Agent::deleteRoute() {
	route.clear();
}

void Agent::setRoute(std::vector<unsigned int> _route) {
	route = _route;
}

void Agent::addRouteStep(unsigned int _dir) {
	route.insert(route.begin(), _dir);
}

unsigned int Agent::getStartX(void) { return StartX; }
unsigned int Agent::getStartY(void) { return StartY; }
unsigned int Agent::getID(void) { return mID; }
std::vector<unsigned int> Agent::getRoute(void) { return route; }
