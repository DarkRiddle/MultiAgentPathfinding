/***************************************************************************
 *                                                                         *
 ***************************************************************************/

#include "wheeliebot_controller.h"

#include <assert.h>
using namespace std;
using namespace matrix;

WheelieBotController::WheelieBotController(const std::string& name, const std::string& revision)
  : AbstractController(name, revision) {
  initialised=false;
  // add threshold parameter to configurable parameters, setable on console
  addParameterDef("threshold", &threshold, 0.2, 0, 1, "threshold for IR-sensor");
}

void WheelieBotController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
  // Very simple controller, if IR sensors are higher than threshold then
  // the robot will turn oposite direction, or backward when to close
  // Sensors index 0 and 1 are wheel speeds
  // Sensors index from 2 to 4 are left front IR sensors
  // from 5 to 7 are the font left infra red sensors (4,5 being in the center)
  // from 8 and 9 are the back sensors
  if (sensors[4] > 2*threshold || sensors[5] > 2*threshold) { // move backward
    motors[0] = -1.;
    motors[1] = -1.;
  }else if (sensors[2] > threshold || sensors[3] > threshold || sensors[4] > threshold) {
    motors[0] = .1;
    motors[1] = 1.;
  }
  else if (sensors[5] > threshold || sensors[6] > threshold || sensors[7] > threshold) {
    motors[0] = 1.;
    motors[1] = .1;
  }
  else { // Move forward
    motors[0] = 1.;
    motors[1] = 1.;
  }
}

void WheelieBotController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  stepNoLearning(sensors,sensornumber, motors, motornumber);
}


void WheelieBotController::init(int sensornumber, int motornumber, RandGen* randGen) {
  assert(motornumber >=2 && sensornumber >=8);
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;
  initialised=true;
}


int WheelieBotController::getSensorNumber() const {
  return nSensors;
}

int WheelieBotController::getMotorNumber() const {
  return nMotors;
}

bool WheelieBotController::store(FILE* f) const {
  return true;
}

bool WheelieBotController::restore(FILE* f) {
  return true;
}

