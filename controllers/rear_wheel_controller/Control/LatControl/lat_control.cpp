/*
 * lat_control.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#include "lat_control.h"

using namespace math;


LatControl::LatControl() {
	// TODO Auto-generated constructor stub
    Init();
}

LatControl::~LatControl() {
	// TODO Auto-generated destructor stub
}

void LatControl::Init()
{
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    _last_cross_err = 0.0f;
	_lat_control_status = init_status;
    common::LpfCoefficients(0.02,10,&den, &num);
    _digital_filter = common::DigitalFilter(den, num);
}

void LatControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{

}

/**
 * @brief LatControl::SatFunction Sat函数
 * @param x：输入
 * @return 返回Sat数值
 */
float LatControl::SatFunction(float x)
{
	if(x > COEFFICIENT_DELTA)
	{
		return  1.0f;
	}
	else if(x < -COEFFICIENT_DELTA)
	{
		return -1.0f;
	}
	else
	{
		return x/COEFFICIENT_DELTA;
	}
}

/**
 * @brief LatControl::ProcV1_0 基于非时间参考的滑模控制算法
 * @param msg：车辆信号反馈，主要获取车辆速度信号
 * @param ctl：车辆控制信号量，控制转向角和角速度信号
 * @param a_track       :实际跟踪变量
 * @param a_track.point :实时跟踪坐标
 * @param a_track.yaw   :实时航向角@(-pi,pi)
 * @param t_track           :目标路径参数变量
 * @param t_track.point     :目标路径与车辆后轴最近的点
 * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
 * @param t_track.curvature :目标路径处的曲率 左正右负
 */
void LatControl::ProcV1_0(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	float cos_psi_a,cos_psi_t;
	float tan_psi_a,tan_psi_t;
	float delta_t,rote_angle;
	float temp_a,temp_b;
	float delta_ctl;

	float yaw_a,yaw_t;
	Vector2d point_a,point_t;

	if((a_track->getYaw() >= -M_PI) && (a_track->getYaw() < -M_3PI4))
	{
		rote_angle = M_3PI4;
	}
	else if((a_track->getYaw() >= -M_3PI4) && (a_track->getYaw() < -M_PI2))
	{
		rote_angle = M_PI2;
	}
	else if((a_track->getYaw() >= -M_PI2) && (a_track->getYaw() < -M_PI4))
	{
		rote_angle = M_PI4;
	}
	else if((a_track->getYaw() >= -M_PI4) && (a_track->getYaw() < 0))
	{
		rote_angle = 0;
	}
	else if((a_track->getYaw() >= 0) && (a_track->getYaw() < M_PI4))
	{
		rote_angle = 0;
	}
	else if((a_track->getYaw() >= M_PI4) && (a_track->getYaw() < M_PI2))
	{
		rote_angle = -M_PI4;
	}
	else if((a_track->getYaw() >= M_PI2) && (a_track->getYaw() < M_3PI4))
	{
		rote_angle = -M_PI2;
	}
    else if((a_track->getYaw() >= M_3PI4) && (a_track->getYaw() <= M_PI))
	{
		rote_angle = -M_3PI4;
	}
	else
	{
        rote_angle = 0;
        return;
	}

	point_a = a_track->getPosition().rotate(rote_angle);
	point_t = t_track.point.rotate(rote_angle);

	yaw_a = a_track->getYaw() + rote_angle;
	yaw_t = t_track.yaw       + rote_angle;

	cos_psi_t = cosf(yaw_t);
	tan_psi_t = tanf(yaw_t);

	cos_psi_a = cosf(yaw_a);
	tan_psi_a = tanf(yaw_a);

	delta_t = atanf(WHEEL_BASE*t_track.curvature);

	_x1 = point_t.getY() - point_a.getY();

	if(Drive == msg->getGear())
	{
		_x2 = tan_psi_t - tan_psi_a;
	}
	else if(Reverse == msg->getGear())
	{
		_x2 = tan_psi_a - tan_psi_t;
	}
	else
	{

	}
	_sliding_variable = COEFFICIENT_SMV * _x1 + _x2;

	temp_a = WHEEL_BASE * cos_psi_a * cos_psi_a * cos_psi_a;
	temp_b = tanf(delta_t)/(WHEEL_BASE * cos_psi_t * cos_psi_t * cos_psi_t) +
			COEFFICIENT_SMV * _x2 +
			COEFFICIENT_RHO * SatFunction(_sliding_variable) +
			COEFFICIENT_K   * _sliding_variable;

	delta_ctl = atanf(temp_a * temp_b);

    delta_ctl = delta_ctl > 0.54f ? 0.54f : delta_ctl < -0.54f ? -0.54f:delta_ctl;
    ctl->setSteeringAngle(_digital_filter.Filter( delta_ctl * 16 * 57.3f));
	ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
}

/**
 * @brief LatControl::RearWheelFeedback 基于车辆后轴位置反馈的路径跟踪算法
 * @param msg：车辆信号反馈，主要获取车辆速度信号
 * @param ctl：车辆控制信号量，控制转向角和角速度信号
 * @param a_track       :实际跟踪变量
 * @param a_track.point :实时跟踪坐标
 * @param a_track.yaw   :实时航向角@(-pi,pi)
 * @param t_track           :目标路径参数变量
 * @param t_track.point     :目标路径与车辆后轴最近的点
 * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
 * @param t_track.curvature :目标路径处的曲率 左正右负
 */
void LatControl::RearWheelFeedback(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	Vector2d vec_d,vec_t;
	float psi_omega = 0.0f;
    float v_x = 0.0f,k = 0.0f;
	float delta_ctl = 0.0f;

    vec_d = a_track->getPosition() - t_track.point;

    if(Drive == msg->getGear())
    {
        vec_t = Vector2d(cosf(t_track.yaw),sinf(t_track.yaw));
        _err_yaw = math::NormallizeAngle(a_track->getYaw() - t_track.yaw);
        v_x = msg->getVehicleMiddleSpeed();
        k =  t_track.curvature;
    }
    else if(Reverse == msg->getGear())
    {
        vec_t = Vector2d(cosf(t_track.yaw + M_PI),sinf(t_track.yaw + M_PI));
        _err_yaw = math::NormallizeAngle(a_track->getYaw() - t_track.yaw - M_PI);
        v_x = msg->getVehicleMiddleSpeed();
        k = -t_track.curvature;
    }
    else
    {
        vec_t = Vector2d(0.0f,0.0f);
        _err_yaw = 0.0f;
        v_x = 0.0f;
    }
    _err_cro = vec_t.CrossProduct(vec_d);
    psi_omega = v_x * k * cosf(_err_yaw)/(1.0f - k * _err_cro)
              - COEFFICIENT_KE   * v_x  * sinf(_err_yaw) * _err_cro / _err_yaw
              - COEFFICIENT_KPSI * fabs(v_x) * _err_yaw;

	if(fabs(psi_omega) < 1.0e-6f || fabs(_err_yaw) < 1.0e-6f)
	{
		ctl->setSteeringAngle(0.0f);
		ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
	}
	else
	{
        delta_ctl = atanf(psi_omega * WHEEL_BASE / v_x);
        delta_ctl = delta_ctl > 0.6 ? 0.6 : delta_ctl < -0.6 ? -0.6:delta_ctl;

        ctl->setSteeringAngle(-_digital_filter.Filter(delta_ctl));
		// ctl->setSteeringAngle(-delta_ctl);

		ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
	}
}


/**
 * @brief LatControl::Work 横向控制状态切换控制
 * @param msg：车辆信息->挡位、车速
 * @param ctl：车辆控制->转向角、转向角速度、车速、挡位
 * @param a_track：当前跟踪位置信息
 * @param t_track：目标曲线信息
 * @param last_track：车辆终点信息
 */
void LatControl::Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TrajectoryAnalyzer *track_aly)
{
    float nerest_distance;
	float init_steering;
	TargetTrack temp_track = track_aly->CalculateNearestPointByPosition(a_track->getPosition().getX() ,
																		a_track->getPosition().getY());
	switch(_lat_control_status)
	{
		case init_status:
			init_steering = atan(WHEEL_BASE*temp_track.curvature);
			ctl->setVelocity(0.0);
			ctl->setGear(Parking);
			if((msg->getSteeringAngle() < (-init_steering + 0.1f)) && (msg->getSteeringAngle() > (-init_steering - 0.1f)))
			{
				printf("steering angle arrive\r\n");
				_lat_control_status = process_status;
			}
			else
			{
				ctl->setSteeringAngle(-init_steering);
				printf("init steering angle:%f\r\n",init_steering);
				ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
			}
			break;

		case process_status:
			nerest_distance = track_aly->DistanceToEnd(a_track->getPosition().getX(),a_track->getPosition().getY());
			if( nerest_distance < 0.2f)
			{
				ctl->setDistance(0);
				ctl->setVelocity(0.0);
				ctl->setGear(Parking);
				_lat_control_status = init_status;
			}
			else
			{
				ctl->setVelocity(10.8);
				ctl->setGear(Drive);
				RearWheelFeedback(msg,ctl,a_track,temp_track);
			}
			break;

		default:
            ctl->setSteeringAngle(0.0);
            ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
            _lat_control_status = init_status;
			break;
	}
}

float LatControl::getX1()            { return  _x1;}
void  LatControl::setX1(float value) { _x1 = value;}

float LatControl::getX2()            { return  _x2;}
void  LatControl::setX2(float value) { _x2 = value;}

float LatControl::getSlidingVariable()            { return  _sliding_variable;}
void  LatControl::setSlidingVariable(float value) { _sliding_variable = value;}

float LatControl::getErrYaw() {return _err_yaw;}
float LatControl::getErrCro() {return _err_cro;}
