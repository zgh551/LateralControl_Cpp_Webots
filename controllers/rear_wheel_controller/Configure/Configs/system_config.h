/*
 * system_config.h
 *
 *  Created on: 2019年3月30日
 *      Author: zhuguohua
 */

#ifndef CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_
#define CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_

/****************Task Shedule Priority Level******************/
#define TIME_5MS_TASK (1)
#define CAN0_TASK     (2)
#define CAN1_TASK     (3)
#define CAN2_TASK     (4)

/****************System Configure******************/
//#define CHANGAN
//#define BORUI
//#define DONG_FENG_E70
#define BMW_X5
#define SIMULATION 0
/********************是否使用超声波避障使能按钮***********************/
#define ULTRASONIC_COLLISION_ENABLE  ( 1 ) // 超声避障使能按钮

// #define M_PI    ( 3.1415926535897932384626433832795f )
#define M_3PI4  ( 2.3561944901923449288469825374596f )
#define M_PI2   ( 1.5707963267948966192313216916398f )
#define M_PI4   ( 0.7853981633974483096156608458198f )

#endif /* CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_ */
