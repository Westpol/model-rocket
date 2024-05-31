#include "Servo.h"

int finpwm[4] = {1500, 1500, 1500, 1500};

Servo fin1;
Servo fin2;
Servo fin3;
Servo fin4;

void setup() {
  Serial.begin(115200);
  fin1.attach(3);
  fin2.attach(4);
  fin3.attach(5);
  fin4.attach(6);
}

void loop() {
  if(Serial.available() > 0){
    char Buffer;
    Buffer = Serial.read();
    if(Buffer == '%'){
      getPackage();
    }
  }
  fin1.writeMicroseconds(finpwm[0]);
  fin2.writeMicroseconds(finpwm[1]);
  fin3.writeMicroseconds(finpwm[2]);
  fin4.writeMicroseconds(finpwm[3]);

}

void getPackage(){
  int servoNum;
  char localBuffer;
  String message = "";

  while(true) {
    if(Serial.available() > 0) {
      servoNum = (int)Serial.read() - 48;
      break;
    }
  }

  while(true){
    if(Serial.available() > 0){
      localBuffer = Serial.read();
      if(localBuffer != ';'){
        if(localBuffer != ','){
          message += localBuffer;
        }
      }
      else {
        finpwm[servoNum] = message.toInt();
        return;
      }
    }
  }
}