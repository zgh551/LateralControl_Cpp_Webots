/*
 * bo_rui_message.cpp
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#include "bmw_message.h"



BMWMessage::BMWMessage() {
	// TODO Auto-generated constructor stub
//	CRC8 crc8 = CRC8(CRC8::eAUTOSAR);
}

BMWMessage::~BMWMessage() {
	// TODO Auto-generated destructor stub
}

void BMWMessage::Init()
{

}

void BMWMessage::Parse(const uint32_t id,const uint8_t *dat,const uint32_t lenght)
{
//	uint8_t crc_temp,i;
//	uint8_t dat_temp[7];
	switch(id)
	{
		case 0x2A0://eps status

			break;

		case 0x122:// wheel speed
			// WheelSpeedFrontRight = ((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S;
			// WheelSpeedFrontLeft  = ((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S;
			break;

		case 0x123:// wheel speed

			break;

		case 0x124://Wheel speed pulse

			break;

		case 0x165:
			
			break;
		case 0x125:// ESC

			break;

		case 0x0E0://SAS

			break;

		default:

			break;
	}
}
