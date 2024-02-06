/*
 * EncoderMotor_interface.h
 *
 *  Created on: Jan 27, 2024
 *      Author: sherief alglaly
 */
#include "stm32f4xx_hal.h"
#ifndef INC_ENCODERMOTOR_INTERFACE_H_
#define INC_ENCODERMOTOR_INTERFACE_H_

typedef struct{
	int16_t velocity;
	int64_t position;
	uint32_t last_counter_value;
}encoder_instance;


#endif /* INC_ENCODERMOTOR_INTERFACE_H_ */
