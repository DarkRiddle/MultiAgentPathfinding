/***************************************************************************
*                                                                         *
***************************************************************************/

// Simulation
#include <ode_robots/simulation.h>
// Agent: bind robot, controller and wiring together
#include <ode_robots/odeagent.h>

#include "wheeliebot.h"
#include "simenvironment.h"

using namespace lpzrobots;

/*class RobotInstance {

}*/

class ThisSim : public Simulation
{
	public:
	ThisSim() { }
	~ThisSim() { }

	/// start() is called at the start and should create all the object (obstacles, agents...).
	virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// Initial position and orientation of the camera (use 'p' in graphical window to find out)
		setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
		// Some simulation parameters can be set here
		global.odeConfig.setParam("controlinterval", 1);
		global.odeConfig.setParam("gravity", -9.8);

		//Adding robots
		WheelieBot* w1 = new WheelieBot("Wheelie1","$ID$",.0,.0,.2,odeHandle,osgHandle,global);
		WheelieBot* w2 = new WheelieBot("Wheelie2","$ID$",2,2,.4,odeHandle,osgHandle,global);
		WheelieBot* w3 = new WheelieBot("Wheelie3","$ID$",5,2,.4,odeHandle,osgHandle,global);
		WheelieBot* w4 = new WheelieBot("Wheelie4","$ID$",6,6,.4,odeHandle,osgHandle,global);

		//creatung simulation environment
		new SimEnvironment(odeHandle,osgHandle,global);

	}

	/* Functions not used in this tutorial but typically useful */
	virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
	}

	virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down) {
		return false;
	}

	virtual void bindingDescription(osg::ApplicationUsage & au) const {
	}
};

int main (int argc, char **argv)
{
	// New simulation
	ThisSim sim;
	// set Title of simulation
	sim.setTitle("NO BASIC SIM by Imara");
	// Simulation begins
	return sim.run(argc, argv) ? 0 : 1;
}




