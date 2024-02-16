/*
 * Position_Control.h
 *
 *  Created on: Feb 4, 2024
 *      Author: sherief alglaly
 */

#ifndef INC_POSITION_CONTROL_H_
#define INC_POSITION_CONTROL_H_

typedef enum
{
	Backward_move,
	Forward_move
}Direction_MainMotor;

typedef enum
{
	up_move,
	down_mov,
	stop
}Dir_suspensionMoto;

typedef enum
{
	Right_motor,
	Left_motor,
	forward_motor,
	Backward_motor
}motor_no;

typedef struct
{
	 motor_no Motor_no;
     uint8_t max_dutyCycle;
	//Pins
	uint8_t Encoder_pin;
	GPIO_TypeDef * Encoder_port;
	uint8_t Direction_pin1;

	uint8_t Encoder_pin_2;
	GPIO_TypeDef * Encoder2_port;


	GPIO_TypeDef * Direction_port1;
	uint8_t Direction_pin2;
	GPIO_TypeDef * Direction_port2;
	uint8_t Motor_Enable;
	uint8_t PWM_channel;
	TIM_HandleTypeDef * PWM_Timer;

	//Direction
	 Direction_MainMotor Direction_main;

	// Suspension motors
	 Dir_suspensionMoto Direction_suspension;

	/******************position*************************/
	int32_t CurrentPosition;
	int32_t Target_position;
    int32_t PrevPosition;
	//howskeeping values
	float prev_err;
	int32_t err;
	float prev_time;
	float errIntegral;
	float errdot;
	float deltaerror;
	//pid
	float proportional;
	float integral;
	float differential;
	//control
	float ControlSignal;


}Motor_t ;

void init_motors(Motor_t* array_of_motors);
/*************position ******************/
void Position_GetNewTarget(Motor_t* motor,uint32_t Targt);
void Position_Calc_Signal(Motor_t* );
void UpdateEncoder(Motor_t* ,Dir_suspensionMoto dir  );
void DriveMotor(Motor_t* );
void DriveMotorDirection(Motor_t* );
void init_PIDVal(Motor_t* motor,float P,float I,float D);


#endif /* INC_POSITION_CONTROL_H_ */
