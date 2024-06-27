/*
Channels:
Throttle = 0
Aileron = 1
Elevator = 2
Roll = 3
Armswitch = 4
*/

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

float neutral_settings[6] = {1001, 1001, 1440, 1590, 1560, 1400};
float servoSetpoints[6] = {neutral_settings[0], neutral_settings[1], neutral_settings[2], neutral_settings[3], neutral_settings[4], neutral_settings[5]};
float servoDirections[4][6] = { {1, 1, 0, 0, 0, 0},     //thrust
                                {0, 0, 0, 0, 1, -1},    //roll
                                {0, 0, -1, 1, 0, 0},     //nick
                                {0, 0, 1, 1, 1, 1} };    //yaw


int motors_idle = 1063;
float gyroX, gyroY, gyroZ;


int sbus_min = 173;
int sbus_max = 1810;

bool armswitch_latch = false;
bool arming_failed = true;

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

  mpu.update();
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();

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

  servoSetpoints[0] = minmax(map(data.ch[0], sbus_min, sbus_max, 1000, 2000), 1000, 2000);   // simple throttle set
  servoSetpoints[1] = minmax(map(data.ch[0], sbus_min, sbus_max, 1000, 2000), 1000, 2000);

                                  // add every control to the mix
  for(int f = 1; f < 4; f++){     // goes through AER of the TAER control axis
    for(int i = 2; i < 6; i++){   // goes through every Servo
      servoSetpoints[i] = minmax(servoSetpoints[i] + (map(data.ch[f], sbus_min, sbus_max, -500, 500) * servoDirections[f][i]), 1000, 2000);
    }
  }

  if (data.ch[4] < 500) {
    for (int i = 0; i < 6; i++) {
      servoSetpoints[i] = neutral_settings[i];
    }
  }
  else if (data.ch[4] > 500 && data.ch[4] < 1500) {
    servoSetpoints[0] = neutral_settings[0];
    servoSetpoints[1] = neutral_settings[1];
  }
  else if (data.ch[4] > 1500){
    servoSetpoints[0] = minmax(servoSetpoints[0], motors_idle, 2000);
    servoSetpoints[1] = minmax(servoSetpoints[1], motors_idle, 2000);
  }

  if(data.ch[4] > 1500 && armswitch_latch == false){
    if(data.ch[0] <= 200){
      armswitch_latch = true;
      arming_failed = false;
    }
    if(data.ch[0] > 200){
      armswitch_latch = true;
      arming_failed = true;
    }
  }

  if(data.ch[4] < 1500 && armswitch_latch == true){
    armswitch_latch = false;
    arming_failed = false;
  }


  if(armswitch_latch == false || arming_failed == true){
    servoSetpoints[0] = neutral_settings[0];
    servoSetpoints[1] = neutral_settings[1];
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