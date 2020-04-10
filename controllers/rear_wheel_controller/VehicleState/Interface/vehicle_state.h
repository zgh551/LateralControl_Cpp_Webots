/*
 * vehicle_state.h
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: vehicle_state.h                     COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#ifndef VEHICLESTATE_VEHICLESTATE_H_
#define VEHICLESTATE_VEHICLESTATE_H_


#include "math.h"
#include "../../Math/vector_2d.h"
#include "../../Configure/Configs/vehicle_config.h"
#include "../../Interaction/CANBUS/Interface/message_manager.h"

class VehicleState
{
public:
	VehicleState();
	virtual ~VehicleState();

	void Init(void);
	
	Vector2d getPosition();
	void     setPosition(Vector2d value);

	float getYaw();
	void  setYaw(float value);

	float getLinearVelocity();
	void  setLinearVelocity(float value);

	float getLinearRate();
	void  setLinearRate(float value);

	float getTurnningRadius();
	void  setTurnningRadius(float value);

	float getPulseUpdateVelocity();
	void  setPulseUpdateVelocity(float value);

	float getAccUpdateVelocity();
	void  setAccUpdateVelocity(float value);
protected:
	Vector2d _position;
	float    _yaw;
	float _linear_velocity;
	float _linear_rate;
	float _turnning_radius;

	float _pulse_update_velocity;
	float _acc_update_velocity;
};

#endif /* VEHICLESTATE_VEHICLESTATE_H_ */
