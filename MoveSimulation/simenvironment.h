/***************************************************************************
 *                                                                         *
 ***************************************************************************/

// Header guard
#ifndef __SIMENVIRONMENT_H
#define __SIMENVIRONMENT_H

#include <ode_robots/oderobot.h>
// Environment and obstacles
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

using namespace lpzrobots;

class SimEnvironment {
	public:
		SimEnvironment(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global);
	private:
		Playground* playground;
		PassiveBox* box;
};
#endif // Header guard
