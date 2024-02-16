/**
  ******************************************************************************
  * @file           : mpu_readings.h
  * @brief          : C file contains all the implementations and FP
  * @author 		: Mahmoud Sayed
  ******************************************************************************
**/

#include "mpu_readings.h"

void MPU_init(void)
{
	   mpu1.data = 0x00;
	   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &mpu1.data, 1, HAL_MAX_DELAY);
	   mpu1.data = 0x08;
	   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &mpu1.data, 1, HAL_MAX_DELAY);
	   mpu1.data = 0x10;
	   HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &mpu1.data, 1, HAL_MAX_DELAY);

	   for(mpu1.i = 0; mpu1.i<2000; mpu1.i++)
	   {
	 	  mpu1.prevtime2 = mpu1.time2;
	 	  mpu1.time2 = HAL_GetTick();
	 	  mpu1.elapsedtime2 = (mpu1.time2 - mpu1.prevtime2)*1000;

	 	  mpu1.cuffer[0] = 0x43;
	 	  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, mpu1.cuffer, 1, HAL_MAX_DELAY);
	 	  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, mpu1.cuffer, 6, HAL_MAX_DELAY);

	 	  mpu1.gyro_raw[0] = (mpu1.cuffer[0] << 8 | mpu1.cuffer[1]);
	 	  mpu1.gyro_raw[1] = (mpu1.cuffer[2] << 8 | mpu1.cuffer[3]);
	 	  mpu1.gyro_raw[2] = (mpu1.cuffer[4] << 8 | mpu1.cuffer[5]);

	 	  mpu1.gyro_cal[0] += mpu1.gyro_raw[0];
	 	  mpu1.gyro_cal[1] += mpu1.gyro_raw[1];
	 	  mpu1.gyro_cal[2] += mpu1.gyro_raw[2];

	 	  HAL_Delay(3);
	   }

	   mpu1.gyro_cal[0] /= 2000;
	   mpu1.gyro_cal[1] /= 2000;
	   mpu1.gyro_cal[2] /= 2000;

	   HAL_Delay(1000);
}

void MPU_run(void)
{
			mpu1.prevtime1 = mpu1.time1;
	 	 	mpu1.time1 = HAL_GetTick();
	 	 	mpu1.elapsedtime1 = (mpu1.time1 - mpu1.prevtime1)*1000;

			 mpu1.tuffer[0] = 0x3B;
			 HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, mpu1.tuffer, 1, HAL_MAX_DELAY);
			 HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, mpu1.tuffer, 6, HAL_MAX_DELAY);

			 // ACC RAW VALUES
			 mpu1.acc_raw[0] = (mpu1.tuffer[0] << 8 | mpu1.tuffer[1]);
			 mpu1.acc_raw[1] = (mpu1.tuffer[2] << 8 | mpu1.tuffer[3]);
			 mpu1.acc_raw[2] = (mpu1.tuffer[4] << 8 | mpu1.tuffer[5]);

			 mpu1.buffer[0] = 0x41;
			 HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, mpu1.buffer, 1, HAL_MAX_DELAY);
			 HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, mpu1.buffer, 2, HAL_MAX_DELAY);


			 //temp values
			 mpu1.raw_temp = (mpu1.buffer[0] << 8 | mpu1.buffer[1]);
			 mpu1.temp = (mpu1.raw_temp / 340.0) + 36.53;


			 mpu1.cuffer[0] = 0x43;
			 HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, mpu1.cuffer, 1, HAL_MAX_DELAY);
			 HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, mpu1.cuffer, 6, HAL_MAX_DELAY);

			 // GYRO RAW VALUES
			 mpu1.gyro_raw[0] = (mpu1.cuffer[0] << 8 | mpu1.cuffer[1]);
			 mpu1.gyro_raw[1] = (mpu1.cuffer[2] << 8 | mpu1.cuffer[3]);
			 mpu1.gyro_raw[2] = (mpu1.cuffer[4] << 8 | mpu1.cuffer[5]);

			 mpu1.gyro_raw[0] -= mpu1.gyro_cal[0];
			 mpu1.gyro_raw[1] -= mpu1.gyro_cal[1];
			 mpu1.gyro_raw[2] -= mpu1.gyro_cal[2];

			 mpu1.angle_pitch_gyro += mpu1.gyro_raw[0] * 0.0000611;
			 mpu1.angle_roll_gyro += mpu1.gyro_raw[1] * 0.0000611;

			 mpu1.angle_pitch_gyro += mpu1.angle_roll_gyro * sin(mpu1.gyro_raw[2] * 0.000001066);
			 mpu1.angle_roll_gyro -= mpu1.angle_pitch_gyro * sin(mpu1.gyro_raw[2] * 0.000001066);

			 mpu1.acc_total_vector = sqrt((mpu1.acc_raw[0]*mpu1.acc_raw[0])+(mpu1.acc_raw[1]*mpu1.acc_raw[1])+(mpu1.acc_raw[2]*mpu1.acc_raw[2]));

			 //57.296 = 1 / (3.412/180)
			 mpu1.angle_pitch_acc = asin((float)mpu1.acc_raw[1]/mpu1.acc_total_vector)* 57.296;
			 mpu1.angle_roll_acc = asin((float)mpu1.acc_raw[0]/mpu1.acc_total_vector)* -57.296;

			 mpu1.angle_pitch_acc -= 0.00;
			 mpu1.angle_roll_acc -= 0.00;

			 //adding low pass filter & high pass filter
			 if(mpu1.set_gyro){
				 mpu1.angle_pitch = mpu1.angle_pitch_gyro * 0.9996 + mpu1.angle_pitch_acc * 0.0004;
				 mpu1.angle_roll  = mpu1.angle_roll_gyro * 0.9996 + mpu1.angle_roll_acc * 0.0004;
			 }
			 else{
				 mpu1.angle_pitch = mpu1.angle_pitch_acc;
				 mpu1.set_gyro = true;
			 }

			 while((HAL_GetTick() - mpu1.prevtime)*1000 < 4000);
			 mpu1.prevtime = HAL_GetTick();
}