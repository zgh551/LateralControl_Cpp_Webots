/*
 * message_manager.h
 *
 *  Created on: December 29 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: message_manager.h                   COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Messege manage module 								         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     December 29 2018    Initial Version                  */
/*****************************************************************************/

#ifndef CANBUS_INTERFACE_MESSAGE_MANAGER_H_
#define CANBUS_INTERFACE_MESSAGE_MANAGER_H_

#include "../../../Utils/type_init.h"

typedef enum _GearStatus
{
    None = 0,
    Parking,
    Reverse,
    Neutral,
    Drive
}GearStatus;

typedef enum _DirectStatus
{
    StandStill = 0,
    Forward,
    Backward,
    Invalid
}DirectStatus;

typedef enum _ActuatorStatus
{
    ActuatorNormal = 0,
    ActuatorErr
}ActuatorStatus;

typedef enum _SpeedStatus
{
    SpeedNormal = 0,
    SpeedAbnormal
}SpeedStatus;

class MessageManager {
public:
    MessageManager();
    virtual ~MessageManager();

    virtual void Init() = 0;
    virtual void Parse(const uint32_t id,const uint8_t *data,const uint32_t lenght) = 0;

    // wheel speed
    float getWheelSpeedFrontLeft();
    void  setWheelSpeedFrontLeft(float value);

    float getWheelSpeedFrontRight();
    void  setWheelSpeedFrontRight(float value);

    float getWheelSpeedRearRight();
    void  setWheelSpeedRearRight(float value);

    float getWheelSpeedRearLeft();
    void  setWheelSpeedRearLeft(float value);

    // the middle speed of the vehicle
    float getVehicleMiddleSpeed();
    void  setVehicleMiddleSpeed(float value);

    // the middle speed valid single
    uint8_t getVehicleMiddleSpeedValid();
    void    setVehicleMiddleSpeedValid(uint8_t value);

    // the middle speed abnormal single
    SpeedStatus getVehicleMiddleSpeedAbnormal();
    void        setVehicleMiddleSpeedAbnormal(SpeedStatus value);

    // wheel speed direction
    DirectStatus getWheelSpeedDirection();
    void         setWheelSpeedDirection(DirectStatus value);

    // wheel pulse
    uint16_t getWheelPulseFrontLeft();
    void     setWheelPulseFrontLeft(uint16_t value);

    uint16_t getWheelPulseFrontRight();
    void     setWheelPulseFrontRight(uint16_t value);

    uint16_t getWheelPulseRearRight();
    void     setWheelPulseRearRight(uint16_t value);

    uint16_t getWheelPulseRearLeft();
    void     setWheelPulseRearLeft(uint16_t value);

    int32_t getWheelSumPulse();
    void    setWheelSumPulse(int32_t value);

    // wheel pulse dirction
    DirectStatus getWheelPulseDirection();
    void         setWheelPulseDirection(DirectStatus value);

    // SAS Steering angle
    float getSteeringAngle();
    void  setSteeringAngle(float value);

    float getSteeringAngleRate();
    void  setSteeringAngleRate(float value);

    // TCU
    GearStatus getGear();
    void       setGear(GearStatus value);

    // ESP Sensor
    float getYawRate();
    void  setYawRate(float value);

    float getLonAcc();
    void  setLonAcc(float value);

    float getLatAcc();
    void  setLatAcc(float value);

    uint8_t getBrakePressure();
    void  setBrakePressure(uint8_t value);

    ActuatorStatus getESC_Status();
    void           setESC_Status(ActuatorStatus value);

    ActuatorStatus getEPS_Status();
    void           setEPS_Status(ActuatorStatus value);
private:
    // wheel speed
    float _wheel_speed_front_left ;
    float _wheel_speed_front_right;
    float _wheel_speed_rear_right ;
    float _wheel_speed_rear_left  ;
    // vehicle speed base pulse,calculate by self
    float   _vehicle_middle_speed;
    uint8_t _vehicle_middle_speed_valid;
    SpeedStatus _vehicle_middle_speed_abnormal;
    // wheel speed direction
    DirectStatus _wheel_speed_direction;

    // wheel pulse
    uint16_t _wheel_pulse_front_left ;
    uint16_t _wheel_pulse_front_right;
    uint16_t _wheel_pulse_rear_right ;
    uint16_t _wheel_pulse_rear_left  ;

    int32_t _wheel_sum_pulse;
    // wheel pulse dirction
    DirectStatus _wheel_pulse_direction;

    // SAS Steering angle
    float _steering_angle;
    float _steering_angle_rate;

    // TCU
    GearStatus _gear;

    // ESP Sensor
    float _yaw_rate;
    float _lon_acc;
    float _lat_acc;

    uint8_t _brake_pressure;

    ActuatorStatus _esc_status;
    ActuatorStatus _eps_status;
};

#endif /* CANBUS_INTERFACE_MESSAGE_MANAGER_H_ */
