/*
 * geometric_track.cpp
 *
 *  Created on: January 9 2019
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: geometric_track.cpp                 COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: track the vehilce position        					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/
#include "geometric_track.h"

VehicleConfig m_GeometricVehicleConfig;

GeometricTrack::GeometricTrack() {
    Init();
}

GeometricTrack::~GeometricTrack() {

}

void GeometricTrack::Init(void)
{
    _position.setX(0);
    _position.setY(0);
    setYaw(0.0f);
    _last_yaw = getYaw();
    setLinearVelocity(0.0f);

    _last_rear_left_pulse   = 0;
    _last_rear_right_pulse  = 0;
    _sum_rear_left_pulse    = 0;
    _sum_rear_right_pulse   = 0;
    _delta_rear_left_pulse  = 0;
    _delta_rear_right_pulse = 0;

     _cumulation_rear_left_pulse  = 0;
     _cumulation_rear_right_pulse = 0;

    _wait_time_cnt = 0;
}

void GeometricTrack::Init(float x,float y,float yaw)
{
    _position.setX(x);
    _position.setY(y);
    setYaw(0.0f);
    _last_yaw = getYaw();
    setLinearVelocity(0.0f);

    _last_rear_left_pulse   = 0;
    _last_rear_right_pulse  = 0;
    _sum_rear_left_pulse    = 0;
    _sum_rear_right_pulse   = 0;
    _delta_rear_left_pulse  = 0;
    _delta_rear_right_pulse = 0;

     _cumulation_rear_left_pulse  = 0;
     _cumulation_rear_right_pulse = 0;

    _wait_time_cnt = 0;
}

/**************************************************************************************/
int32_t GeometricTrack::getSumRearLeftPulse()             { return _sum_rear_left_pulse;}
void    GeometricTrack::setSumRearLeftPulse(int32_t value){_sum_rear_left_pulse = value;}

int32_t GeometricTrack::getSumRearRightPulse()             { return _sum_rear_right_pulse;}
void    GeometricTrack::setSumRearRightPulse(int32_t value){_sum_rear_right_pulse = value;}
