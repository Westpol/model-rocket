#include <SPI.h>
#include "Wire.h"
#include "SoftwareSerial.h"

#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

MPU6050 mpu(Wire);

SoftwareSerial dataBus(10, 9);

float neutral_settings[6] = {1000, 1000, 1440, 1450, 1450, 1400};
float servoSetpoints[6] = {neutral_settings[0], neutral_settings[1], neutral_settings[2], neutral_settings[3], neutral_settings[4], neutral_settings[5]};
float servoDirections[4][6] = { {1, 1, 0, 0, 0, 0},     //thrust
                                {0, 0, 0, 0, 1, -1},    //roll
                                {0, 0, -1, 1, 0, 0},     //nick
                                {-1, 1, 1, 1, 1, 1} };    //yaw


int motors_idle = 1063;
float gyroX, gyroY, gyroZ;
float lastGyroX, lastGyroY, lastGyroZ;
float pid_corrections[4] {0, 0, 0, 0};    // thrust, roll, nick, yaw

float p[4] = {0, 2, 3, 3};
float d[4] = {0, 4, 6, 6};
float i[4] = {0, 0, 0, 0};

void setup() {

  dataBus.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if unable to connect to MPU6050
  delay(2000);
  mpu.calcOffsets(true, true); // gyro and accelero
}

void loop() {

  mpu.update();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
  /*Serial.print(gyroX);
  Serial.print(" | ");
  Serial.print(gyroY);
  Serial.print(" | ");
  Serial.println(gyroZ);*/

  pid_corrections[3] = -gyroX * p[1];    // P-val
  pid_corrections[2] = -gyroY * p[2];
  pid_corrections[1] = gyroZ * p[3];
  //pid_corrections[0] = gyroX * p[0];

  pid_corrections[3] += (lastGyroX - gyroX) * d[1];   // D-val
  pid_corrections[2] += (lastGyroY - gyroY) * d[2];
  pid_corrections[1] += -(lastGyroZ - gyroZ) * d[3];
  //pid_corrections[0] += (lastGyroX - gyroX) * d[0];

  lastGyroX = gyroX;
  lastGyroY = gyroY;
  lastGyroZ = gyroZ;

  calculateServoVals();

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
  dataBus.print((int)servoSetpoints[4]);
  dataBus.print(';');
  dataBus.print("%5,");   // SERVO
  dataBus.print((int)servoSetpoints[5]);
  dataBus.print(';');
}

void calculateServoVals(){

  for(int i = 0; i < 6; i++){
    servoSetpoints[i] = neutral_settings[i];
  }

  servoSetpoints[0] = motors_idle;   // simple throttle set
  servoSetpoints[1] = motors_idle;

                                  // add every control to the mix
  for(int f = 1; f < 4; f++){     // goes through AER of the TAER control axis
    for(int i = 0; i < 6; i++){   // goes through every Servo
      servoSetpoints[i] = minmax(servoSetpoints[i] + (pid_corrections[f] * servoDirections[f][i]), 1000, 2000);
    }
  }
  if(servoSetpoints[0] < motors_idle){
    servoSetpoints[0] = motors_idle;
  }
  if(servoSetpoints[1] < motors_idle){
    servoSetpoints[1] = motors_idle;
  }
}

float minmax(float val, float min, float max){
  if(val < min){
    val = min;
  }
  if(val > max){
    val = max;
  }
  return val;
}