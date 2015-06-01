#include "simenvironment.h"

using namespace osg;
using namespace std;
using namespace lpzrobots;

SimEnvironment::SimEnvironment(const OdeHandle& ode, const OsgHandle& osg, GlobalData& global) {
	/** Environment and obstacles */
	// New playground
	Playground* playground = new Playground(ode, osg, osg::Vec3(15., .2, 1.2), 1);
	// Set colours
	playground->setGroundColor(Color(.784, .784, .0));
	playground->setColor(Color(1., .784, .082, .3));
	// Set position
	playground->setPosition(osg::Vec3(.0, .0, .1));
	// Adding playground to obstacles list
	global.obstacles.push_back(playground);

	// Add a new box obstacle (or use 'o' to drop random obstacles)
	PassiveBox* box = new PassiveBox(ode, osg, osg::Vec3(1., 1., 1.), 2.);
	box->setPose(osg::Matrix::translate(-.5, 4., .7));
	global.obstacles.push_back(box);
}
