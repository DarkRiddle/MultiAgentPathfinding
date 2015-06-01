#include "wheeliebot.h"


using namespace osg;
using namespace std;

WheelieBot::WheelieBot(const std::string& name, const std::string& revision, double pos_x, double pos_y, double pos_z, const OdeHandle& ode, const OsgHandle& osg, GlobalData& global) : robot_name(name), robot_revision(revision) {
	
	conf = WheelieBotBody::getDefaultConf();
	conf.wheelMass=.5;
	robot = new WheelieBotBody(ode, osg, conf, getRobotName());

	((OdeRobot*)robot)->place(Pos(pos_x,pos_y,pos_z));
	controller = new WheelieBotController(getControllerName(), robot_revision);
	wiring = new One2OneWiring(new ColorUniformNoise(.1));
	agent = new OdeAgent(global);

	agent->init(controller, robot, wiring);
	global.agents.push_back(agent);
	global.configs.push_back(agent);
		
}

std::string WheelieBot::getControllerName() {
	std::stringstream ss;
	ss << robot_name << " Controller";
	return ss.str();
}
std::string WheelieBot::getRobotName() {
	std::stringstream ss;
	ss << robot_name << " Robot";
	return ss.str();
}
