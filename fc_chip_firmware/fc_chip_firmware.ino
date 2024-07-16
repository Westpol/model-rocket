/*
Channels:
Throttle = 0
Aileron = 1
Elevator = 2
Roll = 3
Armswitch = 4

Pins:
-------------------
SD Karte
CS = 13
SCK = 10
MOSI = 11
MISO = 12
-------------------
Gyro
SCL = A5
SDA = A4
-------------------
Baro
SCL = A5
SDA = A4
-------------------
XF Nano RX
SBUS = 0
-------------------
Servo Comm
Softwareserial TX = 9
-Unused- Softwareserial RX = 10
-------------------
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

float neutral_settings[6] = {1001, 1001, 1440, 1450, 1450, 1400};
float servoSetpoints[6] = {neutral_settings[0], neutral_settings[1], neutral_settings[2], neutral_settings[3], neutral_settings[4], neutral_settings[5]};
float servoDirections[4][6] = { {1, 1, 0, 0, 0, 0},     //thrust
                                {0, 0, 0, 0, 1, -1},    //roll
                                {0, 0, -1, 1, 0, 0},     //nick
                                {0, 0, 1, 1, 1, 1}};    //yaw WITHOUT motors
                                //{-1, 1, 1, 1, 1, 1}};    //yaw WITH motors


int motors_idle = 1063;
float gyroX, gyroY, gyroZ;
float lastGyroX, lastGyroY, lastGyroZ;


int sbus_min = 173;
int sbus_max = 1810;

bool armswitch_latch = false;
bool arming_failed = true;

float pid_corrections[4] {0, 0, 0, 0};    // thrust, roll, nick, yaw

float p[4] = {0, 2, 3, 3};
float d[4] = {0, 4, 6, 6};
float i[4] = {0, 0, 0, 0};

int roll_deg = 180;    // roll degrees at full stick reflection
int nick_deg = 180;    // roll degrees at full stick reflection
int yaw_deg = 180;    // roll degrees at full stick reflection

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
  gyroX = mpu.getGyroX() + minmax(map(data.ch[3], sbus_min, sbus_max, -yaw_deg, yaw_deg), -yaw_deg, yaw_deg);
  gyroY = mpu.getGyroY() - minmax(map(data.ch[2], sbus_min, sbus_max, -nick_deg, nick_deg), -nick_deg, nick_deg);
  gyroZ = mpu.getGyroZ() + minmax(map(data.ch[1], sbus_min, sbus_max, -roll_deg, roll_deg), -roll_deg, roll_deg);

  pid_controller();

  calculateServoVals();

  sendPackage();
}

void pid_controller(){
  pid_corrections[3] = -gyroX * p[1];    // P-val
  pid_corrections[2] = -gyroY * p[2];
  pid_corrections[1] = gyroZ * p[3];

  pid_corrections[3] += (lastGyroX - gyroX) * d[1];   // D-val
  pid_corrections[2] += (lastGyroY - gyroY) * d[2];
  pid_corrections[1] += -(lastGyroZ - gyroZ) * d[3];
  lastGyroX = gyroX;
  lastGyroY = gyroY;
  lastGyroZ = gyroZ;
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

  servoSetpoints[0] = minmax(map(data.ch[0], sbus_min, sbus_max, 1001, 2000), 1001, 2000);   // simple throttle set
  servoSetpoints[1] = minmax(map(data.ch[0], sbus_min, sbus_max, 1001, 2000), 1001, 2000);

                                  // add every control to the mix
  for(int f = 1; f < 4; f++){     // goes through AER of the TAER control axis
    for(int i = 2; i < 6; i++){   // goes through every Servo
      servoSetpoints[i] = minmax(servoSetpoints[i] + (pid_corrections[f] * servoDirections[f][i]), 1000, 2000);
    }
  }

  arm_handling();
}

void arm_handling(){
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