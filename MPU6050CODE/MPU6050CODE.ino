/*
*****************************************************************************
project name: active suspension system
mini project titie: getting data from the MPU
Authors : 1- Mahmoud Sayed 
          2- Sherief Abdelmohsen 
github repo: https://github.com/ma7moud111/sus_control/tree/main/MPU6050CODE
****************************************************************************
*/

#include <Wire.h>


/*type the length of the half of the car in centimeters*/
#define LENGTH_OF_HALF_OF_CAR_IN_CM   12.5

// define the acceelerometer variables
float accelX, accelY, accelZ;
float angleroll, anglepitch;
float looptimer;

// define the gyrscope variables
long gyroX, gyroY, gyroZ;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float height_of_bump;


void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  int iter;
  for(iter = 0; iter<2000; iter++){
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);    
  }

RateCalibrationPitch /= 2000;
RateCalibrationRoll /= 2000;
RateCalibrationYaw /= 2000;


}

//int last=0;
// int current =0;
// int period=0;


void loop() {
// current = millis();
  gyro_signals();
  print_data();
}

/**********switch on the low pass filter***************/
void gyro_signals(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  //configure the accelerometer outputs
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //pull the accelerometer measurements from the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t accelXLSB = Wire.read() << 8 | Wire.read();
  int16_t accelYLSB = Wire.read() << 8 | Wire.read();
  int16_t accelZLSB = Wire.read() << 8 | Wire.read();

  /*configure the gyroscope output and pull rotation rate measurements from the sensor*/
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x8); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.endTransmission();  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll =(float) GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  // convert the measurements to physical values
  accelX = (float)accelXLSB / 4096 - 0.01;
  accelY = (float)accelYLSB / 4096 + 0.01;
  accelZ = (float)accelZLSB / 4096 + 0.02;

  //calculating theta pitch and theta roll 
  angleroll = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * 1/(3.14/180);
  anglepitch = atan(accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 1/(3.14/180);
  height_of_bump = LENGTH_OF_HALF_OF_CAR_IN_CM * tan(angleroll * 3.14/180);


}

void print_data(void)
{
  Serial.print("Roll angle(deg)= ");
  Serial.print(angleroll);
  Serial.print("   pitch angle(deg)= ");
  Serial.print(anglepitch);
  Serial.print("  h = ");
  Serial.print(height_of_bump);
  Serial.print("\n");
  // period = current -last;
  // last =current;
  delay(100);
}


