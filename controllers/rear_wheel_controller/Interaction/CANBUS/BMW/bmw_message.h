/*
 * bo_rui_message.h
 *
 *  Created on: 2019年3月15日
 *      Author: zhuguohua
 */

#ifndef CANBUS_BORUI_BO_RUI_MESSAGE_H_
#define CANBUS_BORUI_BO_RUI_MESSAGE_H_



#include "../Interface/message_manager.h"
#include "../../../Configure/Configs/vehicle_config.h"

class BMWMessage  : public MessageManager
{
public:
	BMWMessage();
	virtual ~BMWMessage();

	void Init() override;
    void Parse(const uint32_t id,const uint8_t *data,const uint32_t lenght) override;

private:

};

#endif /* CANBUS_BORUI_BO_RUI_MESSAGE_H_ */
