/*
 * VehicleControllercontroller.cpp
 *
 *  Created on: December 28 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: VehicleControllercontroller.cpp               COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: VehicleController control base class       					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 28 2018    Initial Version                  */
/*****************************************************************************/
#include "vehicle_controller.h"

VehicleController::VehicleController() {
    /* ACC */
    _target_acceleration = 0.0f;
    _acceleration = 0.0f;
    _acceleration_enable = 0;

    /* AEB */
    _deceleration = 0.0f;
    _deceleration_enable = 0;

    /* Torque */
    _torque = 0.0f;
    _torque_enable = 0;

    /* Turnning Torque Control Single */
    _turn_torque_val = 0.0f;
    _turn_torque_dir = 0;
    _turn_torque_act = 0;

    /* Velocity */
    _velocity = 0.0f;
    _distance = 0.0f;
    _distance_set = 0.0f;
    _velocity_enable = 0;

    /* Steering System */
    _steering_angle = 0.0f;
    _steering_angle_rate = 0.0f;
    _steering_angle_set = 0.0f;
    _steering_enable = 0;

    /* Gear */
    _gear = 0;
    _gear_enable = 0;

    /* APA */
    _apa_enable = 0;

    /* EPB */
    _epb_enable = 0;
}

VehicleController::~VehicleController() {

}

/// ACC
float VehicleController::getTargetAcceleration()           { return _target_acceleration;}
void  VehicleController::setTargetAcceleration(float value){_target_acceleration = value;}

float VehicleController::getAcceleration()           { return _acceleration;}
void  VehicleController::setAcceleration(float value){_acceleration = value;}

uint8_t VehicleController::getAccelerationEnable()             { return _acceleration_enable;}
void    VehicleController::setAccelerationEnable(uint8_t value){_acceleration_enable = value;}

/// AEB
float VehicleController::getDeceleration()           { return _deceleration;}
void  VehicleController::setDeceleration(float value){_deceleration = value;}

uint8_t VehicleController::getDecelerationEnable()             { return _deceleration_enable;}
void    VehicleController::setDecelerationEnable(uint8_t value){_deceleration_enable = value;}

/// Torque
float VehicleController::getTorque()           { return _torque;}
void  VehicleController::setTorque(float value){_torque = value;}

uint8_t VehicleController::getTorqueEnable()             { return _torque_enable;}
void    VehicleController::setTorqueEnable(uint8_t value){_torque_enable = value;}

/// Turnning Torque Control Single
float VehicleController::getTurnTorqueVal()           { return _turn_torque_val;}
void  VehicleController::setTurnTorqueVal(float value){_turn_torque_val = value;}

uint8_t VehicleController::getTurnTorqueDir()             { return _turn_torque_dir;}
void    VehicleController::setTurnTorqueDir(uint8_t value){_turn_torque_dir = value;}

uint8_t VehicleController::getTurnTorqueAct()             { return _turn_torque_act;}
void    VehicleController::setTurnTorqueAct(uint8_t value){_turn_torque_act = value;}
/// Velocity
float VehicleController::getVelocity()           { return _velocity;}
void  VehicleController::setVelocity(float value){_velocity = value;}
/// Distance
float VehicleController::getDistance()           { return _distance;}
void  VehicleController::setDistance(float value){_distance = value;}

float VehicleController::getDistanceSet()           { return _distance_set;}
void  VehicleController::setDistanceSet(float value){_distance_set = value;}

uint8_t VehicleController::getVelocityEnable()             { return _velocity_enable;}
void    VehicleController::setVelocityEnable(uint8_t value){_velocity_enable = value;}

/// Steering Angle
float VehicleController::getSteeringAngle()           { return _steering_angle;}
void  VehicleController::setSteeringAngle(float value){_steering_angle = value;}

float VehicleController::getSteeringAngleRate()           { return _steering_angle_rate;}
void  VehicleController::setSteeringAngleRate(float value){_steering_angle_rate = value;}

float VehicleController::getSteeringAngleSet()           { return _steering_angle_set;}
void  VehicleController::setSteeringAngleSet(float value){_steering_angle_set = value;}

uint8_t VehicleController::getSteeringEnable()             { return _steering_enable;}
void    VehicleController::setSteeringEnable(uint8_t value){_steering_enable = value;}

/// Gear
uint8_t VehicleController::getGear()             { return _gear;}
void    VehicleController::setGear(uint8_t value){_gear = value;}

uint8_t VehicleController::getGearEnable()             { return _gear_enable;}
void    VehicleController::setGearEnable(uint8_t value){_gear_enable = value;}

/// APA Enable
uint8_t VehicleController::getAPAEnable()             { return _apa_enable;}
void    VehicleController::setAPAEnable(uint8_t value){_apa_enable = value;}

/// EPB Enable
uint8_t VehicleController::getEPBEnable()             { return _epb_enable;}
void    VehicleController::setEPBEnable(uint8_t value){_epb_enable = value;}
