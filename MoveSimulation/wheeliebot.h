/***************************************************************************
 *                                                                         *
 ***************************************************************************/

// Header guard
#ifndef __WHEELIEBOT_H
#define __WHEELIEBOT_H

// Noise Generator
#include <selforg/noisegenerator.h>

// Agent object to bind robot specifics together
#include <ode_robots/odeagent.h>

// WheelieBot specifics
#include "wheeliebot_body.h"
#include "wheeliebot_controller.h"

// Wiring
#include <selforg/one2onewiring.h>

#include <string>

using namespace lpzrobots;

class WheelieBot {
	public:
		WheelieBot(const std::string& name, const std::string& revision, double pos_x, double pos_y, double pos_z, const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global);
	private:
		WheelieBotBodyConf conf;
		AbstractController* controller;
		OdeRobot* robot;
		AbstractWiring* wiring;
		OdeAgent* agent;

		std::string robot_name;
		std::string robot_revision;
		
		std::string getControllerName();
		std::string getRobotName();
};
#endif // Header guard
