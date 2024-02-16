
/**
  ******************************************************************************
  * @file           : mpu_readings.h
  * @brief          : this file contains all the definitions and FP
  * @author 		: Mahmoud Sayed
  ******************************************************************************
**/

#ifndef MPU_READINGS_H_
#define MPU_READINGS_H_

/************************ all includes *****************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/********************* MPU6050 registers ******************/
#define MPU6050_ADDR 	0x68<<1
#define PWR_MGMT_1_REG 	0x6B
#define SMPLRT_DIV_REG 	0x19
#define GYRO_CNFG_REG 	0x1B
#define ACC_CNFG_REG 	0x1C


/**************** MPU typedef *************/
typedef struct {
	uint8_t data;
	uint8_t buffer[2],tuffer[6],cuffer[6];
	int16_t gyro_raw[3],acc_raw[3];
	float gyro_cal[3];
	int16_t acc_total_vector;
	float angle_pitch_gyro,angle_roll_gyro;
	float angle_pitch_acc,angle_roll_acc;
	float angle_pitch,angle_roll;
	int16_t raw_temp;
	float temp;
	int i;
	float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2;
	HAL_StatusTypeDef set_gyro;
}MPU_vars_t;

MPU_vars_t  mpu1;


/************************ MPU FP ************************/
void MPU_init(void);
void MPU_run(void);

#endif
