#include <SPI.h>
#include "Wire.h"
#include "SoftwareSerial.h"

#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include "sbus.h"

MPU6050 mpu(Wire);

SoftwareSerial dataBus(10, 9);

bfs::SbusRx sbus_rx(&Serial);
bfs::SbusData data;

float offsets[6] = {0, 0, 0, 0, 0, 0};
float servoSetpoints[6] = {0, 0, 1500, 1500, 1500, 1500};
float servoDirectionsYaw[6] = {0, 0, 1, 1, 1, 1};
float gyroX, gyroY, gyroZ;


void setup() {

  sbus_rx.Begin();

  dataBus.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if unable to connect to MPU6050
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
}

void loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
  }

  /*mpu.update();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
  float p_value = -(gyroX * 2); 
  for(int i = 0; i <= 3; i++){
    servoSetpoints[i] = minmax(middles[i] + (p_value * servoDirectionsYaw[i]), 1100, 1900);
  }*/

  servoSetpoints[0] = map(data.ch[0], 0, 2000, 1000, 2000);
  servoSetpoints[1] = map(data.ch[0], 0, 2000, 1000, 2000);
  sendPackage();
}

void sendPackage(){
  dataBus.print("%0,");   // MOTOR
  dataBus.print((int)servoSetpoints[0]);
  dataBus.print(';');
  dataBus.print("%1,");   // MOTOR
  dataBus.print((int)servoSetpoints[1]);
  dataBus.print(';');
  dataBus.print("%2,");   // SERVO
  dataBus.print((int)servoSetpoints[2]);
  dataBus.print(';');
  dataBus.print("%3,");   // SERVO
  dataBus.print((int)servoSetpoints[3]);
  dataBus.print(';');
  dataBus.print("%4,");   // SERVO
  dataBus.print((int)servoSetpoints[3]);
  dataBus.print(';');
  dataBus.print("%5,");   // SERVO
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