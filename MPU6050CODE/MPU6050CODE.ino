/*

project name: active sus
Authors : Mahmoud Sayed & Sherief Abdelmohsen 

*/

#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  //setupMPU();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  for(int i = 0; i<2000; i++){
    setupMPU();
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
  setupMPU();
  printData();
  // period = current -last;
  // last =current;
  delay(250);
}

void setupMPU(){
  Wire.beginTransmission(0x68); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x05); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.endTransmission();
  Wire.write(0x68); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x08); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.endTransmission();  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = GyroX/65.5;
  RatePitch = GyroY/65.5;
  RateYaw = GyroZ/65.5;

  
   
}



void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(RateRoll-RateCalibrationRoll);
  Serial.print(" Y=");
  Serial.print(RatePitch-RateCalibrationPitch);
  Serial.print(" Z=");
  Serial.print(RateYaw-RateCalibrationYaw);
  Serial.print("\n");
  
}
