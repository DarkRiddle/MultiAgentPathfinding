#include "PathFinder.hpp"
#include "Agent.hpp"

using namespace std;

PathFinder::PathFinder(int n, int m) : map_w(n), map_h(m) {
			map = new int*[map_h];
			closed_nodes_map = new int*[map_h];
			open_nodes_map = new int*[map_h];
			dir_map = new int*[map_h];
			for(int r=0;r<map_h;r++) {
				map[r] = new int[map_w];
				open_nodes_map[r] = new int[map_w];
				closed_nodes_map[r] = new int[map_w];
				dir_map[r] = new int[map_w];
			}
			initMap();
			route = std::vector<unsigned int>();
			testRoute = std::vector<unsigned int>();
			listAgents = std::vector<Agent*>();
};

void PathFinder::setStart(int a_x, int a_y, int b_x, int b_y) {
	AxStart = a_x;
	AyStart = a_y;
	BxStart = b_x;
	ByStart = b_y;
}

void PathFinder::setTarget(int _x, int _y) {
	xObj = _x;
	yObj = _y;
}

unsigned int PathFinder::getTargetX() {
	return xObj;
}

unsigned int PathFinder::getTargetY() {
	return yObj;
}

void PathFinder::Find(){
	Agent* a=nullptr;
	for(int i=0; i<listAgents.size(); i++) {
		a = listAgents.at(i);
		if(a!=nullptr) {
			cout << "Agent: " << a->getID() << endl;
			setAgent(a);
			AStar();
		}
	}
}

void PathFinder::setAgent(Agent* A){
	startX=A->getStartX();
	startY=A->getStartY();
	robotID=A->getID();
	cout<<"start coordinates: "<<startX<<":"<<startY<<endl;
}

void PathFinder::addAgent(Agent* A){
	listAgents.push_back(A);
}

bool PathFinder::Test(int tx, int ty){
	int l;
	static int j;
	int x = tx;
	int y =ty;
	int directions=8;
	testRoute.clear();
	
	testMap = new int*[map_h];
	for(int r=0;r<map_h;r++) {
		testMap[r] = new int[map_w];
	}

	while(!(x==startX && y==startY)){
		j=dir_map[y][x];
		testRoute.insert(testRoute.begin(),((j+directions/2)%directions));
		x+=dx[j];
		y+=dy[j];
	}
	
	if(testRoute.size()>0){	
		for(unsigned int i=0;i<testRoute.size();i++){
			l=testRoute.at(i);
			x=x+dx[l];
			y=y+dy[l];
			testMap[y][x]=3;
		}
		
		int step = testRoute.size();
		
		int lastX, lastY;
		for(int y=0;y<map_h;y++){
			for(int x=0;x<map_w;x++){
				if(testMap[y][x]!=0){
					lastX=x;
					lastY=y;
				}
			}
		}

		for(int z=0; z<robotID; z++){
			Agent* A;
			A=listAgents.at(z);
			oldMap = new int*[map_h];
			for(int r=0;r<map_h;r++) {
				oldMap[r] = new int[map_w];
			}
			x=A->getStartX();
			y=A->getStartY();
			
			for(unsigned int i=0;i<A->getRoute().size();i++){
				l=A->getRoute().at(i);
				x=x+dx[l];
				y=y+dy[l];
				oldMap[y][x]=3;
			}

			if(oldMap[lastY][lastX]==3){
				return true;
			}
		}
	}
	

	return false;
}

void PathFinder::AStar() {
	static priority_queue<Node> pq[2];
	static int pqi;

	static Node* n0;
	static Node* m0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	static int directions=8;
	pqi=0;
	int walkRobot=1;
	int NUMROBOTS=2;
	int test=0;
	bool collideTest=false;
	Agent* currentAgent = listAgents.at(robotID);
	
	for(y=0;y<map_h;y++) {
		for(x=0;x<map_w;x++) {
			closed_nodes_map[y][x]=0;
			open_nodes_map[y][x]=0;
		}
	}

    n0=new Node(startX, startY, 0, 0);
    n0->updatePriority(xObj, yObj);
    pq[pqi].push(*n0);

	open_nodes_map[startY][startX]=n0->getPriority();
	
	delete n0;
    
    //AStar
    while(!pq[pqi].empty()) {
		n0=new Node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
		x=n0->getxPos(); y=n0->getyPos();
		pq[pqi].pop();
		open_nodes_map[y][x]=0;
		closed_nodes_map[y][x]=1;

		if(x==xObj && y==yObj){
			while(!(x==startX && y==startY)){
				j=dir_map[y][x];
				currentAgent->addRouteStep((j+directions/2)%directions);
				x+=dx[j];
				y+=dy[j];
			}
			delete n0;
			while(!pq[pqi].empty()) pq[pqi].pop();
			return;
		}

		for(i=0;i<directions;i++){
			xdx=x+dx[i]; ydy=y+dy[i];
			if(!(xdx<0 || xdx>map_w-1 || ydy<0 || ydy>map_h-1 || map[ydy][xdx]==1 || closed_nodes_map[ydy][xdx]==1)){
				collideTest=false;
				if(currentAgent->getID()!=0){
					collideTest=Test(x,y);
				}
				test++;
				if(!collideTest){
					m0=new Node( xdx, ydy, n0->getLevel(), n0->getPriority());
					m0->nextLevel(i);
					m0->updatePriority(xObj, yObj);

					if(open_nodes_map[ydy][xdx]==0){
						open_nodes_map[ydy][xdx]=m0->getPriority();
						pq[pqi].push(*m0);
						dir_map[ydy][xdx]=(i+directions/2)%directions;
					} else if(open_nodes_map[ydy][xdx]>m0->getPriority()){
						open_nodes_map[ydy][xdx]=m0->getPriority();
						dir_map[ydy][xdx]=(i+directions/2)%directions;
						
						while(!(pq[pqi].top().getxPos()==xdx && pq[pqi].top().getyPos()==ydy)){                
							pq[1-pqi].push(pq[pqi].top());
							pq[pqi].pop();       
						}
						pq[pqi].pop();

						if(pq[pqi].size()>pq[1-pqi].size()){
							pqi=1-pqi;
						}
						while(!pq[pqi].empty()){                
							pq[1-pqi].push(pq[pqi].top());
							pq[pqi].pop();       
						}
						pqi=1-pqi;
						pq[pqi].push(*m0);
					}
							
					else delete m0;
				}
			}
		}
		delete n0;

    }
    return;
}

void PathFinder::initMap(void){
    for(int y=0;y<map_h;y++){
        for(int x=0;x<map_w;x++) map[y][x]=0;
    }
    for(int x=map_w/8;x<map_w*7/8;x++){
        map[map_h/2][x]=1;
    }
    for(int y=map_h/8;y<map_h*7/8;y++){
        map[y][map_w/2]=1;
    }
}

void PathFinder::printPath(){
	Agent* A;
	for(int z=0; z<=robotID; z++){
		A=listAgents.at(z);
		cout<<"Robot ID in printPath: "<<z<<endl;
		int x,y,l;
		
		x=A->getStartX();
		y=A->getStartY();
		map[y][x]=2;
		for(unsigned int i=0;i<A->getRoute().size();i++){
			l=A->getRoute().at(i);
			cout<<A->getRoute().at(i);
			x=x+dx[l];
			y=y+dy[l];
			map[y][x]=z+4;
		}
		cout<<endl;
		map[y][x]=3;
	}

	for(int y=0;y<map_h;y++){
		for(int x=0;x<map_w;x++){
			if(map[y][x]==0) {
				cout<<".";
			} else if(map[y][x]==1) {
				cout<<"O"; //obstacle
			} else if(map[y][x]==2) {
				cout<<"S"; //start
			} else if(map[y][x]==3) {
				cout<<"F"; //finish
			} else {						//Probably more robots
				cout<<(map[y][x]-3);		//Up to 9 robots won't give a problem, after that it won't be able to be printed neatly
			}
		}
		cout<<endl;
	}
}
