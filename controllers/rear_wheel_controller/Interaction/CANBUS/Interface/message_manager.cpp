/*
 * message_manager.c
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: message_manager.c                   COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Messege manage module 								         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/

#include "./Interaction/CANBUS/Interface/message_manager.h"

MessageManager::MessageManager() {
    // wheel speed
    _vehicle_middle_speed = 0.0f;
    _steering_angle = 0.0f;
    _steering_angle_rate = 0.0f;
    _gear = Parking;

}

MessageManager::~MessageManager() {

}


// wheel speed
float MessageManager::getWheelSpeedFrontLeft()           { return _wheel_speed_front_left;}
void  MessageManager::setWheelSpeedFrontLeft(float value){_wheel_speed_front_left = value;}

float MessageManager::getWheelSpeedFrontRight()           { return _wheel_speed_front_right;}
void  MessageManager::setWheelSpeedFrontRight(float value){_wheel_speed_front_right = value;}

float MessageManager::getWheelSpeedRearRight()           { return _wheel_speed_rear_right;}
void  MessageManager::setWheelSpeedRearRight(float value){_wheel_speed_rear_right = value;}

float MessageManager::getWheelSpeedRearLeft()           { return _wheel_speed_rear_left;}
void  MessageManager::setWheelSpeedRearLeft(float value){_wheel_speed_rear_left = value;}

float MessageManager::getVehicleMiddleSpeed()           { return _vehicle_middle_speed;}
void  MessageManager::setVehicleMiddleSpeed(float value){_vehicle_middle_speed = value;}

uint8_t MessageManager::getVehicleMiddleSpeedValid()             { return _vehicle_middle_speed_valid;}
void    MessageManager::setVehicleMiddleSpeedValid(uint8_t value){_vehicle_middle_speed_valid = value;}

SpeedStatus MessageManager::getVehicleMiddleSpeedAbnormal()             { return _vehicle_middle_speed_abnormal;}
void    MessageManager::setVehicleMiddleSpeedAbnormal(SpeedStatus value){_vehicle_middle_speed_abnormal = value;}
// wheel speed direction
DirectStatus MessageManager::getWheelSpeedDirection()                  { return _wheel_speed_direction;}
void         MessageManager::setWheelSpeedDirection(DirectStatus value){_wheel_speed_direction = value;}

// wheel pulse
uint16_t MessageManager::getWheelPulseFrontLeft()              { return _wheel_pulse_front_left;}
void     MessageManager::setWheelPulseFrontLeft(uint16_t value){_wheel_pulse_front_left = value;}

uint16_t MessageManager::getWheelPulseFrontRight()              { return _wheel_pulse_front_right;}
void     MessageManager::setWheelPulseFrontRight(uint16_t value){_wheel_pulse_front_right = value;}

uint16_t MessageManager::getWheelPulseRearRight()              { return _wheel_pulse_rear_right;}
void     MessageManager::setWheelPulseRearRight(uint16_t value){_wheel_pulse_rear_right = value;}

uint16_t MessageManager::getWheelPulseRearLeft()              { return _wheel_pulse_rear_left;}
void     MessageManager::setWheelPulseRearLeft(uint16_t value){_wheel_pulse_rear_left = value;}

int32_t MessageManager::getWheelSumPulse()              { return _wheel_sum_pulse;}
void    MessageManager::setWheelSumPulse(int32_t value){_wheel_sum_pulse = value;}
// wheel pulse dirction
DirectStatus MessageManager::getWheelPulseDirection()                  { return _wheel_pulse_direction;}
void         MessageManager::setWheelPulseDirection(DirectStatus value){_wheel_pulse_direction = value;}

// SAS Steering angle
float MessageManager::getSteeringAngle()           { return _steering_angle;}
void  MessageManager::setSteeringAngle(float value){_steering_angle = value;}

float MessageManager::getSteeringAngleRate()           { return _steering_angle_rate;}
void  MessageManager::setSteeringAngleRate(float value){_steering_angle_rate = value;}

// TCU gear
GearStatus MessageManager::getGear()                { return _gear;}
void       MessageManager::setGear(GearStatus value){_gear = value;}

// ESP Sensor
float MessageManager::getYawRate()           { return _yaw_rate;}
void  MessageManager::setYawRate(float value){_yaw_rate = value;}

float MessageManager::getLonAcc()           { return _lon_acc;}
void  MessageManager::setLonAcc(float value){_lon_acc = value;}

float MessageManager::getLatAcc()           { return _lat_acc;}
void  MessageManager::setLatAcc(float value){_lat_acc = value;}

uint8_t MessageManager::getBrakePressure()             { return _brake_pressure;}
void    MessageManager::setBrakePressure(uint8_t value){_brake_pressure = value;}

ActuatorStatus MessageManager::getESC_Status()                    { return _esc_status;}
void           MessageManager::setESC_Status(ActuatorStatus value){_esc_status = value;}

ActuatorStatus MessageManager::getEPS_Status()                    { return _eps_status;}
void           MessageManager::setEPS_Status(ActuatorStatus value){_eps_status = value;}
