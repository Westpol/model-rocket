#include "Wire.h"
#include <MPU6050_light.h>
#include "SoftwareSerial.h"

MPU6050 mpu(Wire);
SoftwareSerial dataBus(10, 9);

float setpoint = 0;
float middles[4] = {1500, 1500, 1500, 1500};
float servoSetpoints[4] = {middles[0], middles[1], middles[2], middles[3]};
float servoDirectionsYaw[4] = {1, 1, 1, 1};
float gyroX;


void setup() {
  dataBus.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  gyroX = mpu.getGyroX();
  float p_value = setpoint - (gyroX * 2); 
  for(int i = 0; i <= 3; i++){
    servoSetpoints[i] = minmax(middles[i] + (p_value * servoDirectionsYaw[i]), 1100, 1900);
  }
  sendPackage();
}

void sendPackage(){
  dataBus.print("%0,");
  dataBus.print((int)servoSetpoints[0]);
  dataBus.print(';');
  dataBus.print("%1,");
  dataBus.print((int)servoSetpoints[1]);
  dataBus.print(';');
  dataBus.print("%2,");
  dataBus.print((int)servoSetpoints[2]);
  dataBus.print(';');
  dataBus.print("%3,");
  dataBus.print((int)servoSetpoints[3]);
  dataBus.print(';');
}

int minmax(int val, int min, int max){
  if(val < min){
    val = min;
  }
  if(val > max){
    val = max;
  }
  return val;
}