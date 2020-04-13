// File:          target_pen_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Pen.hpp>

#include "../rear_wheel_controller/Planning/Curvature/curvature.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

double translation[3];
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Supervisor *robot = new Supervisor();
  int color, red, green, blue;
  Curvature  *curvature_object = new Curvature();
  std::vector<TargetTrack> *_target_curvature_vectors = new std::vector<TargetTrack>;

  curvature_object->GenerateCurvaturePointSets(_target_curvature_vectors,1);
  std::vector<TargetTrack>::iterator it = _target_curvature_vectors->begin();
  translation[0] = 0.0;
  translation[1] = 0.09;
  translation[2] = 0.0;
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  timeStep = 2;
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Node  *target_pen_node = robot->getFromDef("TARGET_PEN");
  Field *pen_translation_field = target_pen_node->getField("translation");

  Pen *pen_robot  = robot->getPen("pen_robot");
  red = 0;
  green = 0xff;
  blue = 0;
  color = (((red << 8) | green) << 8) | blue;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    // pen_robot->setInkColor(color,0.8);
    // pen_robot->write(true);
    // Process sensor data here.
    if(it != _target_curvature_vectors->end())
    {
      translation[2] = it->point.getX();
      translation[0] = it->point.getY();
      pen_translation_field->setSFVec3f(translation);
      it++;
    }
    
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
