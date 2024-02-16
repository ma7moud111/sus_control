/*
 * Position_control.c
 *
 *  Created on: Feb 4, 2024
 *      Author: sherief alglaly
 */
#include "stm32f4xx_hal.h"
#include "Position_Control.h"

void init_motors(Motor_t* motors )
{

//	for (uint8_t i=0;i<4;i++)
//	{
		__HAL_TIM_SET_COMPARE(motors->PWM_Timer, motors->PWM_channel, 0);
		HAL_TIM_PWM_Start(motors->PWM_Timer, motors->PWM_channel);
		motors->ControlSignal=0;
		motors->CurrentPosition=0;
		motors->Direction_suspension=up_move;
		motors->PrevPosition=0;
		motors->Target_position=0;
		motors->differential=0;
		motors->err=0;
		motors->errIntegral=0;
		motors->errdot=0;
		motors->integral=0;
		motors->prev_err=0;
		motors->prev_time=0;
		motors->proportional=0;
		motors->deltaerror=0;
		motors->max_dutyCycle=100;  // by default

	//}
}

void Position_GetNewTarget(Motor_t* motor,uint32_t Targt) // Get new target triggered lastly by mpu readings.
{
	//Target is number of ticks.
	motor->Target_position = Targt;
}
void Position_Calc_Signal(Motor_t* motor) //calc every 2.048ms triggered by half callback interrupt
{
    float current_time = 0.002048;



   if (motor->Target_position > motor->CurrentPosition)
      {
	   motor->Direction_suspension = up_move;
	   motor->err = motor->Target_position - motor->CurrentPosition  ;
      }
   else if (motor->Target_position < motor->CurrentPosition)
     {
	   motor->Direction_suspension = down_mov;
	   motor->err =   motor->CurrentPosition -motor->Target_position ;
     }
   else
   {
	    motor->PrevPosition = motor->Target_position;  // completes mission
	    motor ->Direction_suspension = stop;
	    motor->errIntegral =0;

   }
   if (motor->err >5)
   {
    motor->errIntegral = motor->errIntegral + motor->err * current_time; //2.048ms
   }
    motor->deltaerror =  (motor->err - motor->prev_err) ;
   motor->errdot = motor->deltaerror / current_time;
   motor->ControlSignal = (motor->proportional * motor -> err )+ (motor->errdot * motor-> differential ) +( motor->integral * motor->errIntegral);
   motor->prev_err = motor->err;
   DriveMotor(motor);
}
void UpdateEncoder(Motor_t* motor,Dir_suspensionMoto dir )
{

	if (dir == up_move)
    	motor->CurrentPosition++;
	else if (dir == down_mov)
		motor->CurrentPosition--;

}
void DriveMotor(Motor_t* motor )
{

   DriveMotorDirection(motor);
   if (motor->ControlSignal < 0)
	   motor->ControlSignal=-motor->ControlSignal;

   //abs takes integer.
   uint32_t absoluteIntegerControl = (uint32_t)(motor->ControlSignal);

   if (absoluteIntegerControl > motor-> max_dutyCycle)
   __HAL_TIM_SET_COMPARE(motor->PWM_Timer, motor->PWM_channel,motor-> max_dutyCycle);
   else
   { if (motor->err < 5 || absoluteIntegerControl < 5)
	   {
	     absoluteIntegerControl = 0;
	     motor ->Direction_suspension = stop;
	   }
	  __HAL_TIM_SET_COMPARE(motor->PWM_Timer, motor->PWM_channel, absoluteIntegerControl);
   }

}
void DriveMotorDirection(Motor_t* motor )
{

	if (motor ->Direction_suspension == up_move)
	{
		HAL_GPIO_WritePin(motor->Direction_port1, motor->Direction_pin1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->Direction_port2, motor->Direction_pin2,GPIO_PIN_RESET);
	}
	else if  (motor ->Direction_suspension == down_mov)
	{
		HAL_GPIO_WritePin(motor->Direction_port1, motor->Direction_pin1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->Direction_port2, motor->Direction_pin2,GPIO_PIN_SET);
	}
	else if  (motor ->Direction_suspension == stop)
	{
		HAL_GPIO_WritePin(motor->Direction_port1, motor->Direction_pin1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->Direction_port2, motor->Direction_pin2,GPIO_PIN_RESET);
	}
}
void init_PIDVal(Motor_t* motor,float P,float I,float D)
{
	motor->proportional=P;
	motor->integral=I;
	motor->differential =D;
}
