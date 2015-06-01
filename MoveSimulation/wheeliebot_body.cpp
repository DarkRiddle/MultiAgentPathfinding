/***************************************************************************
 *                                                                         *
 ***************************************************************************/

#include "wheeliebot_body.h"

// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

  WheelieBotBody::WheelieBotBody(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const WheelieBotBodyConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "revision 1.0"), conf(conf){
  }


  WheelieBotBody::~WheelieBotBody(){
  }


  void WheelieBotBody::place(const Matrix& pose){
    // Configuration check: wheels have to be bigger than the body
    assert(2. * conf.wheelRadius > conf.bodyHeight);
    // Movig robot upward so wheel are not stuck on the ground
    Matrix initialPose;
    initialPose = Matrix::translate(Vec3(0, 0, conf.wheelRadius) * pose);
    // Creating the robot
    create(initialPose);
  }

  void WheelieBotBody::create(const Matrix& pose) {
    /* Creating body */
    // Cylinder geometry primitive as body
    Primitive* body = new Cylinder(conf.bodyRadius, conf.bodyHeight);
    // Setting texture from Image library
    //body->setTexture("Images/purple_velour.jpg");
    body->setTexture("Images/rabbit_fur.jpg");
    // Initializing the primitive
    body->init(odeHandle, conf.bodyMass, osgHandle);
    // Setting the pose of the primitive
    body->setPose(pose);
    // Adding the primitive to the list of objects
    objects.push_back(body);

    /* Creating the left wheel */
    Primitive* lWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
    // Setting texture from Images library
    lWheel->setTexture("Images/chess.rgb");
    lWheel->init(odeHandle, conf.wheelMass, osgHandle);
    // The cylinder is rotated 90º on the Y axis
    // then translated outside the radius of the body plus half of
    // its own height
    // -- All transformations have to be relative to the position so
    // the robot can be initialised in any position --
    Matrix lWheelPose =
      Matrix::rotate(M_PI / 2.0, 0, 1, 0) *
      Matrix::translate(conf.bodyRadius + conf.wheelHeight / 2.0, .0, .0) *
      pose;
    // Setting the pose of the wheel
    lWheel->setPose(lWheelPose);
    // adding the wheel to the list of objects
    objects.push_back(lWheel);
    // Joining the wheel to the body by a hingejoint
    // the anchor comes from the wheel and the axis of rotation
    // is relative to the pose of the left wheel
    HingeJoint* bodyLeftWheelJoint = new HingeJoint(body, lWheel,
                                                    lWheel->getPosition(),
                                                    Axis(0, 0, 1) * lWheelPose);
    // Initializing the joint
    bodyLeftWheelJoint->init(odeHandle, osgHandle);
    // Adding the joint to the list of joints
    joints.push_back(bodyLeftWheelJoint);

    /* Creating the right wheel */
    // Analog to left wheel but changing translation direction
    Primitive* rWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
    rWheel->setTexture("Images/chess.rgb");
    rWheel->init(odeHandle, conf.wheelMass, osgHandle);
    Matrix rWheelPose = Matrix::rotate(M_PI / 2.0, 0, 1, 0) *
      Matrix::translate(-(conf.bodyRadius + conf.wheelHeight / 2.0), .0, .0) *
      pose;
    rWheel->setPose(rWheelPose);
    objects.push_back(rWheel);
    HingeJoint* bodyRightWheelJoint = new HingeJoint(body, rWheel,
                                                     rWheel->getPosition(),
                                                     Axis(0, 0, 1) * rWheelPose);
    bodyRightWheelJoint->init(odeHandle, osgHandle);
    joints.push_back(bodyRightWheelJoint);

    /* Motors */
    // Left wheel motor, the OdeHandle, the joint and the maximun
    // power that motor will be used to achieve desired speed
    AngularMotor1Axis* lWheelMotor = new AngularMotor1Axis(odeHandle, bodyLeftWheelJoint,
                                                           conf.wheelMotorPower);
    wheelMotors.push_back(lWheelMotor);
    // Right wheel motor
    AngularMotor1Axis* rWheelMotor = new AngularMotor1Axis(odeHandle, bodyRightWheelJoint,
                                                           conf.wheelMotorPower);
    wheelMotors.push_back(rWheelMotor);

    /* Infra-red sensors */
    // Initialising infra-red sensor bank
    irSensorBank.init(odeHandle, osgHandle);
    // New infra-red sensor array
    IRSensor* irSensor[7];
    // Registering the sensor in the bank, fixed to body
    // For the first sensor it is rotated to point frontward
    // translation from center of body to outside and middle of height
    // pose is relative to the parent body - no need to multiply by 'pose'.
    // Maximum range of sensor value.
    // drawAll will display a line and the sensor body in the rendered scene.
    irSensor[0] = new IRSensor();
    irSensorBank.registerSensor(irSensor[0], body,
                                Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
                                Matrix::rotate( M_PI / 2.0, 0, 0, 1) *
                                Matrix::translate(-conf.bodyRadius * sin(M_PI * .4),
                                                  conf.bodyRadius * cos(M_PI * .4), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[1] = new IRSensor();
    irSensorBank.registerSensor(irSensor[1], body,
                                Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
                                Matrix::rotate( M_PI / 3.5, 0, 0, 1) *
                                Matrix::translate(-conf.bodyRadius * sin(M_PI * .25),
                                                  conf.bodyRadius * cos(M_PI * .25), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[2] = new IRSensor();
    irSensorBank.registerSensor(irSensor[2], body,
                                Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
                                Matrix::translate(-conf.bodyRadius * sin(M_PI * .05),
                                                  conf.bodyRadius * cos(M_PI * .05), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[3] = new IRSensor();
    irSensorBank.registerSensor(irSensor[3], body,
                                Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
                                Matrix::translate(conf.bodyRadius * sin(M_PI * .05),
                                                  conf.bodyRadius * cos(M_PI * .05), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[4] = new IRSensor();
    irSensorBank.registerSensor(irSensor[4], body,
                                Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
                                Matrix::rotate(-M_PI / 3.5, 0, 0, 1) *
                                Matrix::translate(conf.bodyRadius * sin(M_PI * .25),
                                                  conf.bodyRadius * cos(M_PI * .25), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[5] = new IRSensor();
    irSensorBank.registerSensor(irSensor[5], body,
                                Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
                                Matrix::rotate(-M_PI / 2.0, 0, 0, 1) *
                                Matrix::translate(conf.bodyRadius * sin(M_PI * .4),
                                                  conf.bodyRadius * cos(M_PI * .4), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[6] = new IRSensor();
    irSensorBank.registerSensor(irSensor[6], body,
                                Matrix::rotate(M_PI / 2.0, 1, 0, 0) *
                                Matrix::translate(-conf.bodyRadius * sin(M_PI * .9),
                                                  conf.bodyRadius * cos(M_PI * .9), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

    irSensor[7] = new IRSensor();
    irSensorBank.registerSensor(irSensor[7], body,
                                Matrix::rotate(M_PI / 2.0, 1, 0, 0) *
                                Matrix::translate(conf.bodyRadius * sin(M_PI * .9),
                                                  conf.bodyRadius * cos(M_PI * .9), conf.bodyHeight / 2.0),
                                conf.irRange,
                                RaySensor::drawAll);

  }


  int WheelieBotBody::getSensors(sensor* sensors, int sensorNumber){
    // We must keep track of how many sensors we are reading
    int n = 0;
    // Speed sensors from the left wheel motors
    sensors[0] = wheelMotors[0]->get(0);
    n++;
    // Speed sensors from the right wheel motors
    sensors[1] = wheelMotors[1]->get(0);
    n++;
    // IR sensors
    n += irSensorBank.get(sensors + n, sensorNumber - n);

    return n;
  }


  void WheelieBotBody::setMotors(const motor* motors, int motorNumber){
    // Setting the motor command, first argument of set() is ignored
    // (only one axis) second argument is the speed, motors[] values
    // are expected to be bounded to [-1.0, 1.0]
    wheelMotors[0]->set(0, motors[0] * conf.wheelMotorMaxSpeed);
    wheelMotors[1]->set(0, motors[1] * conf.wheelMotorMaxSpeed);
  }


  int WheelieBotBody::getSensorNumber(){
    int n = 0;
    // One sensor for each motor
    n += wheelMotors.size();
    // Infra-red sensors
    n += irSensorBank.size();

    return n;
  }


  int WheelieBotBody::getMotorNumber(){
    return wheelMotors.size();
  }


  void WheelieBotBody::update() {
    // Updating all objects
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++) {
      if(*i) (*i)->update();
    }
    // Updating all joints
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++) {
      if(*i) (*i)->update();
    }
    // Update infra-red sensor bank
    irSensorBank.update();
  }


  void WheelieBotBody::doInternalStuff(GlobalData& globalData){
    // Reset sensorbank (infrared sensors)
    irSensorBank.reset();
  }
}
