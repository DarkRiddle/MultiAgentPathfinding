#include <iostream>

#include "Node.hpp"
#include "PathFinder.hpp"
#include "Agent.hpp"

using namespace std;

int MapSize_w=60;
int MapSize_h=60;

int AxStart=0;
int AyStart=0;
int BxStart=10;
int ByStart=3;
int CxStart=7;
int CyStart=10;
int DxStart=45;
int DyStart=7;

int xObj=59;
int yObj=31;

int xGoal=30;
int yGoal=57;

int main(int argc, char** argv) {
	cout << "Pre-object creation" << endl;	
	PathFinder pf(MapSize_w,MapSize_h);
	Agent agentOne(AxStart,AyStart);
	Agent agentTwo(BxStart,ByStart);
	Agent agentThree(CxStart,CyStart);
	Agent agentFour(DxStart,DyStart);
	
	pf.addAgent(&agentOne);
	pf.addAgent(&agentTwo);
	pf.addAgent(&agentThree);
	pf.addAgent(&agentFour);

	cout << "Post-object creation" << endl;
	pf.setTarget(xObj,yObj);
	
	clock_t start = clock();
	pf.Find();
	clock_t end = clock();
	
	double time_elapsed = double(end - start);
	cout<<"Time to calculate the route(s) (ms): "<<time_elapsed<<endl;

	pf.printPath();
	
	cout<<"Moving Object"<<endl;
	pf.initMap();

	agentOne.deleteRoute();
	agentTwo.deleteRoute();
	agentThree.deleteRoute();
	agentFour.deleteRoute();
	
	agentOne.setStart(pf.getTargetX(),pf.getTargetY());
	agentTwo.setStart(pf.getTargetX(),pf.getTargetY());
	agentThree.setStart(pf.getTargetX(),pf.getTargetY());
	agentFour.setStart(pf.getTargetX(),pf.getTargetY());
	
	pf.setTarget(xGoal,yGoal);
		
	start = clock();
	pf.Find();
	end = clock();
	
	time_elapsed = double(end - start);
	cout<<"Time to calculate the route(s) (ms): "<<time_elapsed<<endl;

	pf.printPath();
	
	return 0;
}
