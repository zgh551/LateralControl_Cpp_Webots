/*
 * bo_rui_controller.cpp
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#include "bmw_controller.h"

BMWController::BMWController() {
    // TODO Auto-generated constructor stub

}

BMWController::~BMWController() {
    // TODO Auto-generated destructor stub
}

void BMWController::Init()
{

}
/**
 * @brief start the vehicle controller.
 * @return true if successfully started.
 */
void BMWController::Start()
{

}

/**
 * @brief stop the vehicle controller.
 */
void BMWController::Stop()
{

}
  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
void BMWController::Update(ControlCommand cmd)
{

}

void BMWController::Update(APAControlCommand cmd)
{
    // APAEnable         = cmd.ControlEnable.B.APAEnable;

    // Gear              = cmd.Gear;

    // SteeringAngle 	  = cmd.SteeringAngle;
    // SteeringAngleRate = cmd.SteeringAngleRate;

    // Velocity          = cmd.Velocity;
    // Distance          = cmd.Distance;
}


void BMWController::VehicleContorl()
{

}

void BMWController::VehicleContorlPri()
{

}

double BMWController::SteeringAngleControl(double wheel_angle)
{
    if(wheel_angle - getSteeringAngleSet() > getSteeringAngleRate())
    {
        wheel_angle = getSteeringAngleSet() + getSteeringAngleRate();
    }
    else if(wheel_angle - getSteeringAngleSet() < -getSteeringAngleRate())
    {
        wheel_angle = getSteeringAngleSet() - getSteeringAngleRate();
    }
    setSteeringAngleSet(wheel_angle);
    return wheel_angle;
}
// void BMWController::SteeringAngleControl(float dt)
// {
    // float da = getSteeringAngleRate() * dt;
    // float left_target_angle = getSteeringAngle() - da;
    // float right_target_angle = getSteeringAngle() + da;

    // if(getSteeringAngleSet() < left_target_angle)
    // {
    //     setSteeringAngleSet(getSteeringAngleSet() + da);
    // }
    // else if(getSteeringAngleSet() > right_target_angle)
    // {
    //     setSteeringAngleSet(getSteeringAngleSet() + da);
    // }
    // else
    // {
    //     setSteeringAngleSet(getSteeringAngle());
    // }
    // if(getSteeringAngle() - getSteeringAngleSet() > getSteeringAngleRate())
    // {

    // }
// }

// void BMWController::SteeringAngleControl(float dt,float actual_steering)
// {

// }

void BMWController::Push(float dt)
{
//	SteeringAngleControl(dt);
//	VehicleContorl();
}

void BMWController::Push(float dt,float actual_steering)
{
    // SteeringAngleControl(dt,actual_steering);
    // VehicleContorl();
}
