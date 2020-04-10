/*
 * vehicle_state.c
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_state.cp                    COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#include "vehicle_state.h"

VehicleState::VehicleState() {

	Init();
}

VehicleState::~VehicleState() {

}

void VehicleState::Init()
{

}

Vector2d VehicleState::getPosition()              { return  _position;}
void     VehicleState::setPosition(Vector2d value){ _position = value;}

float VehicleState::getYaw()           { return _yaw;}
void  VehicleState::setYaw(float value){_yaw = value;}

float VehicleState::getLinearVelocity()           { return _linear_velocity;}
void  VehicleState::setLinearVelocity(float value){_linear_velocity = value;}

float VehicleState::getLinearRate()           { return _linear_rate;}
void  VehicleState::setLinearRate(float value){_linear_rate = value;}

float VehicleState::getTurnningRadius()           { return _turnning_radius;}
void  VehicleState::setTurnningRadius(float value){_turnning_radius = value;}

float VehicleState::getPulseUpdateVelocity()           { return _pulse_update_velocity;}
void  VehicleState::setPulseUpdateVelocity(float value){_pulse_update_velocity = value;}

float VehicleState::getAccUpdateVelocity()           { return _acc_update_velocity;}
void  VehicleState::setAccUpdateVelocity(float value){_acc_update_velocity = value;}
