// File:          rear_wheel_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <string.h>
#include <vector>

#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/vehicle/Driver.hpp>


#include "Configure/Configs/vehicle_config.h"
#include "Interaction/CANBUS/BMW/bmw_controller.h"
#include "Interaction/CANBUS/BMW/bmw_message.h"
#include "VehicleState/GeometricTrack/geometric_track.h"
#include "Planning/Curvature/curvature.h"
#include "Control/LatControl/lat_control_lqr.h"
#include "Control/LatControl/lat_control.h"
#include "Control/Common/trajectory_analyzer.h"

// All the webots classes are defined in the "webots" namespace
/**
 * @brief name space
 */
using namespace webots;
// using namespace math;
#define RED   0x880000
#define GREEN 0x008800
#define BLUE  0x000088
/**
 * @brief Driver Objecter
 */
Car *car;

/**
 * @brief BMW vehicle message
 */
BMWMessage *wb_bmw_message;

/**
 * @brief BMW vehicle control
 */
BMWController *wb_bmw_controller;

/**
 * @brief BMW vehicle location information
 */
GeometricTrack *wb_bmw_track;

/**
 * gennerate target curvature
 */
Curvature *wb_target_curvature;
std::vector<TargetTrack> *wb_target_curvature_vectors;

TrajectoryAnalyzer *wb_trajectory_analyzer;

LatControl_LQR *wb_lat_control_lqr;
LatControl *wb_lat_control;

VehicleConfig *wb_vehicle_configure;
/**
 * @brief car node and relation field
 */
Node  *vehicle;
Field *translationField;
Field *rotationField;
const double *translation_array;
const double *rotation_array;

/**
 * @brief init the variable
 */
void Init(void)
{
  wb_bmw_controller = new BMWController();
  wb_bmw_message = new BMWMessage();
  wb_bmw_track = new GeometricTrack();
  wb_target_curvature = new Curvature();
  wb_target_curvature_vectors = new std::vector<TargetTrack>;

  wb_vehicle_configure = new VehicleConfig();

  wb_trajectory_analyzer = new TrajectoryAnalyzer();

  wb_lat_control_lqr = new LatControl_LQR();
  wb_lat_control_lqr->Init(wb_vehicle_configure);

  wb_lat_control = new LatControl();
  wb_lat_control->Init();
}

/**
 * @brief Display Function
 * @param none
 * @return none
 */
void UpdateDisplay(BMWMessage *msg,GeometricTrack *ps)
{
  char txt[128];
  sprintf(txt,"X:%.2f[m]  Y:%.2f[m]  Yaw:%.2f[rad] ",ps->getPosition().getX(),ps->getPosition().getY(),ps->getYaw());
  car->setLabel(0,txt,0.05,0.95,0.07,GREEN,0,"Arial");

  sprintf(txt,"Speed:%.2f[m/s]  Steering:%.2f[deg]",msg->getVehicleMiddleSpeed()/3.6,-msg->getSteeringAngle()*16.0*57.3);
  car->setLabel(1,txt,0.05,0.91,0.07,BLUE,0,"Arial");

//  sprintf(txt,"LatErr:%.2f[cm] HeadErr:%.2f[deg]\r\n",wb_lat_control_lqr->getLatError()->getLateralError()*100,
//                                                      wb_lat_control_lqr->getLatError()->getHeadingError()*57.3);
  sprintf(txt,"LatErr:%.2f[cm] HeadErr:%.2f[deg]\r\n",wb_lat_control->getErrCro() *100 ,
                                                      wb_lat_control->getErrYaw() *57.3);
                                                  
  car->setLabel(2,txt,0.05,0.87,0.07,RED,0,"Arial");
}

/** 
 * @brief Get vehicle body information
 * @param msg vehicle message information
 */
void GetVehicleMessage(BMWMessage *msg)
{
  // printf("speed:%f\r\n",car->getCurrentSpeed());
  msg->setVehicleMiddleSpeed(car->getCurrentSpeed());
  msg->setSteeringAngle(car->getSteeringAngle());

  // printf("wheel_base:%f\r\n",car->getWheelbase());
  // if(car->getGear() == 0)
  // {
    msg->setGear(Drive);
  // }
  // else
  // {
  //   msg->setGear(Reverse);
  // }
  
}

/**
 * @brief get the vehicle location information
 * @param ps the vehicle position 
 */
void GetVehicleLocation(GeometricTrack *ps)
{
  translation_array = translationField->getSFVec3f();
  rotation_array = rotationField->getSFRotation();
  ps->setPosition(Vector2d(translation_array[2],translation_array[0]));
  ps->setYaw(rotation_array[3]);
}

/**
 * @brief set the vehicle control command
 */
void SetVehicleInformation(BMWController *ctl)
{
  car->setCruisingSpeed(ctl->getVelocity());
  car->setSteeringAngle(ctl->SteeringAngleControl(ctl->getSteeringAngle()));
  car->setBrakeIntensity(0.0);
  // printf("control steering:%f\r\n",ctl->getSteeringAngle());
}

void VehicleInit()
{
  car->setCruisingSpeed(0.0);
  car->setSteeringAngle(0.0);
  car->setBrakeIntensity(0.0);
}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  car = new Car();

  // init relation variable
  Init();

  VehicleInit();
  // gennerate target curvature
  wb_target_curvature->GenerateCurvaturePointSets(wb_target_curvature_vectors, 3);

  wb_trajectory_analyzer->Init(wb_target_curvature_vectors);

  // get the time step of the current world.
  // int timeStep = (int)car->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // get BMW Solid Node and get the field of translation,rotation
  vehicle = car->getFromDef("BMW");
  translationField = vehicle->getField("translation");
  rotationField = vehicle->getField("rotation"); 

  int mode = car->getControlMode();
  printf("Control mode:%d\r\n",mode);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (car->step() != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    GetVehicleMessage(wb_bmw_message);
    GetVehicleLocation(wb_bmw_track);
    // Process sensor data here.
    // LQR Control
    //wb_lat_control_lqr->Work(wb_bmw_message,wb_bmw_track,wb_trajectory_analyzer,wb_bmw_controller);

    // Rear Wheel Feedback control
    wb_lat_control->Work(wb_bmw_message,wb_bmw_controller,wb_bmw_track,wb_trajectory_analyzer);

    // Enter here functions to send actuator commands, like:
    SetVehicleInformation(wb_bmw_controller);
    UpdateDisplay(wb_bmw_message,wb_bmw_track);
  };

  // Enter here exit cleanup code.

  delete car;
  return 0;
}
