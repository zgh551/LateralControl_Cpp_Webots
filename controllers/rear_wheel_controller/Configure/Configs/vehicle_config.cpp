/*
 * vehilce_config.cpp
 *
 *  Created on: 2019年1月10日
 *      Author: zhuguohua
 */
/*****************************************************************************/
/* FILE NAME: vehilce_config.cpp                  COPYRIGHT (c) Motovis 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: configure the vehile information and the steering and radius */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 10 2019     Initial Version                  */
/* 1.1	 Guohua Zhu     May     13 2019     Steering angle calculatge        */
/*****************************************************************************/
#include "vehicle_config.h"

VehicleConfig::VehicleConfig() {
	Init();
}

VehicleConfig::~VehicleConfig() {

}

void VehicleConfig::Init()
{
	uint8_t i;

	// FrontDiagonalAxis = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(FRONT_EDGE_TO_CENTER,2));
	// RearDiagonalAxis  = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(REAR_EDGE_TO_CENTER,2));

	// FrontDiagonalAngle = atanf(LEFT_EDGE_TO_CENTER/FRONT_EDGE_TO_CENTER);
	// RearDiagonalAngle  = atanf(LEFT_EDGE_TO_CENTER/REAR_EDGE_TO_CENTER);

    // _front_left_diagonal.Length = FrontDiagonalAxis;
    // _front_left_diagonal.Angle  = FrontDiagonalAngle;

    // _front_right_diagonal.Length =  FrontDiagonalAxis;
    // _front_right_diagonal.Angle  = -FrontDiagonalAngle;

    // _rear_left_diagonal.Length = RearDiagonalAxis;
    // _rear_left_diagonal.Angle  = -RearDiagonalAngle;

    // _rear_right_diagonal.Length =  RearDiagonalAxis;
    // _rear_right_diagonal.Angle  =  RearDiagonalAngle;

	// _ultrasonic_location_array[0].Point.X = SENSOR1_X;
	// _ultrasonic_location_array[0].Point.Y = SENSOR1_Y;
	// _ultrasonic_location_array[0].Angle   = SENSOR1_ANGLE;

	// _ultrasonic_location_array[1].Point.X = SENSOR2_X;
	// _ultrasonic_location_array[1].Point.Y = SENSOR2_Y;
	// _ultrasonic_location_array[1].Angle   = SENSOR2_ANGLE;

	// _ultrasonic_location_array[2].Point.X = SENSOR3_X;
	// _ultrasonic_location_array[2].Point.Y = SENSOR3_Y;
	// _ultrasonic_location_array[2].Angle   = SENSOR3_ANGLE;

	// _ultrasonic_location_array[3].Point.X = SENSOR4_X;
	// _ultrasonic_location_array[3].Point.Y = SENSOR4_Y;
	// _ultrasonic_location_array[3].Angle   = SENSOR4_ANGLE;

	// _ultrasonic_location_array[4].Point.X = SENSOR5_X;
	// _ultrasonic_location_array[4].Point.Y = SENSOR5_Y;
	// _ultrasonic_location_array[4].Angle   = SENSOR5_ANGLE;

	// _ultrasonic_location_array[5].Point.X = SENSOR6_X;
	// _ultrasonic_location_array[5].Point.Y = SENSOR6_Y;
	// _ultrasonic_location_array[5].Angle   = SENSOR6_ANGLE;

	// _ultrasonic_location_array[6].Point.X = SENSOR7_X;
	// _ultrasonic_location_array[6].Point.Y = SENSOR7_Y;
	// _ultrasonic_location_array[6].Angle   = SENSOR7_ANGLE;

	// _ultrasonic_location_array[7].Point.X = SENSOR8_X;
	// _ultrasonic_location_array[7].Point.Y = SENSOR8_Y;
	// _ultrasonic_location_array[7].Angle   = SENSOR8_ANGLE;

	// _ultrasonic_location_array[8].Point.X = SENSOR9_X;
	// _ultrasonic_location_array[8].Point.Y = SENSOR9_Y;
	// _ultrasonic_location_array[8].Angle   = SENSOR9_ANGLE;

	// _ultrasonic_location_array[9].Point.X = SENSOR10_X;
	// _ultrasonic_location_array[9].Point.Y = SENSOR10_Y;
	// _ultrasonic_location_array[9].Angle   = SENSOR10_ANGLE;

	// _ultrasonic_location_array[10].Point.X = SENSOR11_X;
	// _ultrasonic_location_array[10].Point.Y = SENSOR11_Y;
	// _ultrasonic_location_array[10].Angle   = SENSOR11_ANGLE;

	// _ultrasonic_location_array[11].Point.X = SENSOR12_X;
	// _ultrasonic_location_array[11].Point.Y = SENSOR12_Y;
	// _ultrasonic_location_array[11].Angle   = SENSOR12_ANGLE;

	/*
	 * 初始化扭矩标定表
	 * */

	_acc_num = ACC_ARRAY_NUM;
	_vlc_num  = VELOCITY_ARRAY_NUM;
	for(i=0;i<ACC_ARRAY_NUM;i++)
	{
		_accelerate_table[i] = acc_table[i];
	}
	for(i=0;i<VELOCITY_ARRAY_NUM;i++)
	{
		_velocity_table[i] = velocity_table[i];
	}
	for(i=0;i< (ACC_ARRAY_NUM * VELOCITY_ARRAY_NUM);i++)
	{
		_torque_table[i] = torque_table[i/VELOCITY_ARRAY_NUM][i%VELOCITY_ARRAY_NUM];
	}

    _ts = TS;
    _cf = CF;
    _cr = CR;
    _mass_fl = MASS_FL;
    _mass_fr = MASS_FR;
    _mass_rl = MASS_RL;
    _mass_rr = MASS_RR;
    _eps = EPS;
    _max_iteration = MAX_ITERATION;

    _matrix_q1 = MATRIX_Q1;
    _matrix_q2 = MATRIX_Q2;
    _matrix_q3 = MATRIX_Q3;
    _matrix_q4 = MATRIX_Q4;

    _matrix_q[0] = MATRIX_Q1;
    _matrix_q[1] = MATRIX_Q2;
    _matrix_q[2] = MATRIX_Q3;
    _matrix_q[3] = MATRIX_Q4;

    _wheel_base = WHEEL_BASE;
    _steering_ratio = STEERING_RATIO;
    _max_steering_angle = MAX_STEERING_ANGLE;
    _max_steering_angle_rate = MAX_STEERING_ANGLE_RATE;
    _min_speed_protection = MIN_SPEED_PROTECTION;
}

// r is + and -
void VehicleConfig::EdgeRadius(float r)
{
	// RadiusFrontLeft = sqrtf( powf( r - LEFT_EDGE_TO_CENTER , 2 ) +
	// 		                 powf( FRONT_EDGE_TO_CENTER , 2 ) );

	// RadiusFrontRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
    //                           powf( FRONT_EDGE_TO_CENTER , 2 ) );

	// RadiusRearLeft = sqrtf( powf( r -  LEFT_EDGE_TO_CENTER , 2 ) +
	// 		                powf( REAR_EDGE_TO_CENTER , 2 ) );

	// RadiusRearRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
    //                          powf( REAR_EDGE_TO_CENTER , 2 ) );
}

float VehicleConfig::SteeringAngle2TurnningRadius(float steer,float a,float b)
{
    return steer < 0 ? -WHEEL_BASE / tanf((a * -steer + b) * PI / 180.0f) :
                        WHEEL_BASE / tanf((a *  steer + b) * PI / 180.0f) ;
}

float VehicleConfig::SteeringAngle2TurnningRadiusExp(float steer,float a,float b)
{
	return steer < 0 ? -a * expf(b * -steer) : a * expf(b * steer);
}

float VehicleConfig::TurnningRadius2SteeringAngleExp(float radius,float a,float b)
{
	return radius < 0 ? -logf(-radius/a)/b : logf(radius/a)/b;
}

float VehicleConfig::TurnningRadius2SteeringAngle(float radius,float a,float b)
{
	return radius < 0 ? -(atanf(-WHEEL_BASE/radius) * 180 / PI - b)/a :
			             (atanf( WHEEL_BASE/radius) * 180 / PI - b)/a ;
}

float VehicleConfig::TurnRadiusCurveFitting(float steering_angle)
{
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A1,FIT_RADIUS_B1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A2,FIT_RADIUS_B2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A3,FIT_RADIUS_B3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A4,FIT_RADIUS_B4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A5,FIT_RADIUS_B5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A6,FIT_RADIUS_B6) :
			steering_angle >  -50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A7,FIT_RADIUS_B7) :
			steering_angle > -100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A8,FIT_RADIUS_B8) :
			steering_angle > -200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A9,FIT_RADIUS_B9) :
			steering_angle > -300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A10,FIT_RADIUS_B10) :
			steering_angle > -400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A11,FIT_RADIUS_B11) :
									SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A12,FIT_RADIUS_B12);
}

float VehicleConfig::TurnRadiusFindingTable(float steering_angle)
{
    return steering_angle >=  0 ? SteerAngle2Radius[static_cast<uint16_t>(steering_angle)][0] : -SteerAngle2Radius[static_cast<uint16_t>(-steering_angle)][1];
}

float VehicleConfig::SteeringAngleCurveFitting(float radius)
{
	return
           radius >   48.308f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A6,FIT_RADIUS_B6) :
           radius >   24.018f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A5,FIT_RADIUS_B5) :
           radius >   11.910f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A4,FIT_RADIUS_B4) :
           radius >   7.6736f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A3,FIT_RADIUS_B3) :
           radius >    5.463f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A2,FIT_RADIUS_B2) :
           radius >      0.0f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A1,FIT_RADIUS_B1) :
           radius > - 5.4745f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A12,FIT_RADIUS_B12) :
           radius > - 7.7214f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A11,FIT_RADIUS_B11):
           radius > - 11.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A10,FIT_RADIUS_B10):
           radius > - 24.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A9,FIT_RADIUS_B9):
           radius > - 49.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A8,FIT_RADIUS_B8):
							   TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A7,FIT_RADIUS_B7);
}

float VehicleConfig::SteeringAngleFindingCallback(uint16_t left_id,uint16_t right_id,float radius)
{
	uint16_t middle_id;
	middle_id = (left_id + right_id) * 0.5;
	if(radius > 0)
	{
		if( (right_id - left_id) > 1)
		{
			if(radius < SteerAngle2Radius[middle_id][0])
			{
				return SteeringAngleFindingCallback(middle_id,right_id,radius);
			}
			else if(radius > SteerAngle2Radius[middle_id][0])
			{
				return SteeringAngleFindingCallback(left_id,middle_id,radius);
			}
			else
			{
				return middle_id;
			}
		}
		else
		{
            return (left_id + right_id) * 0.5f;
		}
	}
	else
	{
		if( (right_id - left_id) > 1)
		{
			if(-radius < SteerAngle2Radius[middle_id][1])
			{
				return SteeringAngleFindingCallback(middle_id,right_id,radius);
			}
			else if(-radius > SteerAngle2Radius[middle_id][1])
			{
				return SteeringAngleFindingCallback(left_id,middle_id,radius);
			}
			else
			{
				return -middle_id;
			}
		}
		else
		{
            return -(left_id + right_id) * 0.5f;
		}
	}
}

float VehicleConfig::SteeringAngleFindingLoop(uint16_t left_id,uint16_t right_id,float radius)
{
	uint16_t middle_id;
	if(radius > 0)
	{
		while( (right_id - left_id) > 1)
		{
			middle_id = (uint16_t)((left_id + right_id) * 0.5);
			if(radius < SteerAngle2Radius[middle_id][0])
			{
				left_id = middle_id;
			}
			else if(radius > SteerAngle2Radius[middle_id][0])
			{
				right_id = middle_id;
			}
			else
			{
				return middle_id;
			}
		}
        return (left_id + right_id) * 0.5f;
	}
	else
	{
		while( (right_id - left_id) > 1)
		{
            middle_id = (uint16_t)((left_id + right_id) * 0.5f);
			if(-radius < SteerAngle2Radius[middle_id][1])
			{
				left_id = middle_id;
			}
			else if(-radius > SteerAngle2Radius[middle_id][1])
			{
				right_id = middle_id;
			}
			else
			{
				return -middle_id;
			}
		}
        return -(left_id + right_id) * 0.5f;
	}
}

float VehicleConfig::TurnRadiusCalculate(float steering_angle)
{
	return TurnRadiusFindingTable(steering_angle);
}

float VehicleConfig::SteeringAngleCalculate(float radius)
{
	return SteeringAngleFindingCallback(MIN_ARRAY_ID,MAX_ARRAY_ID,radius);
//	return SteeringAngleFindingLoop(MIN_ARRAY_ID,MAX_ARRAY_ID,radius);
}

float VehicleConfig::getRadiusFrontLeft()           { return  _radius_front_left;}
void  VehicleConfig::setRadiusFrontLeft(float value){ _radius_front_left = value;}
float VehicleConfig::getRadiusFrontRight()           { return  _radius_front_right;}
void  VehicleConfig::setRadiusFrontRight(float value){ _radius_front_right = value;}
float VehicleConfig::getRadiusRearLeft()           { return  _radius_rear_left;}
void  VehicleConfig::setRadiusRearLeft(float value){ _radius_rear_left = value;}
float VehicleConfig::getRadiusRearRight()           { return  _radius_rear_right;}
void  VehicleConfig::setRadiusRearRight(float value){ _radius_rear_right = value;}

double VehicleConfig::getFrontDiagonalAxis()           { return  _front_diagonal_axis;}
void  VehicleConfig::setFrontDiagonalAxis(double value){ _front_diagonal_axis = value;}
double VehicleConfig::getFrontDiagonalAngle()           { return  _front_diagonal_angle;}
void  VehicleConfig::setFrontDiagonalAngle(double value){ _front_diagonal_angle = value;}
double VehicleConfig::getRearDiagonalAxis()           { return  _rear_diagonal_axis;}
void  VehicleConfig::setRearDiagonalAxis(double value){ _rear_diagonal_axis = value;}
double VehicleConfig::getRearDiagonalAngle()           { return  _rear_diagonal_angle;}
void  VehicleConfig::setRearDiagonalAngle(double value){ _rear_diagonal_angle = value;}

Location* VehicleConfig::getUltrasonicLocationArray() { return  _ultrasonic_location_array;}

Polar VehicleConfig::getFrontLeftDiagonal()  { return  _front_left_diagonal;}
Polar VehicleConfig::getFrontRightDiagonal() { return  _front_right_diagonal;}
Polar VehicleConfig::getRearLeftDiagonal()   { return  _rear_left_diagonal;}
Polar VehicleConfig::getRearRightDiagonal()  { return  _rear_right_diagonal;}

float* VehicleConfig::getAccelerateTable() { return  _accelerate_table;}
float* VehicleConfig::getVelocityTable() { return  _velocity_table;}
float* VehicleConfig::getTorqueTable() { return  _torque_table;}

uint16_t VehicleConfig::getAccNum() { return  _acc_num;}
uint16_t VehicleConfig::getVlcNum() { return  _vlc_num;}

double VehicleConfig::getTs() { return  _ts;}
double VehicleConfig::getCf() { return  _cf;}
double VehicleConfig::getCr() { return  _cr;}
double VehicleConfig::getMassFl() { return  _mass_fl;}
double VehicleConfig::getMassFr() { return  _mass_fr;}
double VehicleConfig::getMassRl() { return  _mass_rl;}
double VehicleConfig::getMassRr() { return  _mass_rr;}
double VehicleConfig::getEps() { return  _eps;}
uint16_t VehicleConfig::getMaxIteration() { return  _max_iteration;}
double VehicleConfig::getMatrixQ1() { return  _matrix_q1;}
double VehicleConfig::getMatrixQ2() { return  _matrix_q2;}
double VehicleConfig::getMatrixQ3() { return  _matrix_q3;}
double VehicleConfig::getMatrixQ4() { return  _matrix_q4;}
double *VehicleConfig::getMatrixQ() { return  _matrix_q;}

double VehicleConfig::getWheelBase() { return  _wheel_base;}
double VehicleConfig::getSteeringRatio(){ return  _steering_ratio;}
double VehicleConfig::getMaxSteeringAngle(){ return  _max_steering_angle;}
double VehicleConfig::getMaxSteeringAngleRate(){ return  _max_steering_angle_rate;}

double VehicleConfig::getMinSpeedProtection(){ return _min_speed_protection; }
