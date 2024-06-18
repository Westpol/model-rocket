#include "Servo.h"

int finpwm[6] = {1000, 1000, 0, 0, 0, 0};

Servo fin1;
Servo fin2;
Servo fin3;
Servo fin4;
Servo fin5;
Servo fin6;

unsigned long timeoutCounter;

void setup() {
  Serial.begin(115200);
  fin1.attach(2);   // first Stack
  fin2.attach(3);
  fin3.attach(4);

  fin4.attach(6);   // second Stack
  fin5.attach(7);
  fin6.attach(8);
}

void loop() {
  if(Serial.available() > 0){
    char Buffer;
    Buffer = Serial.read();
    if(Buffer == '%'){
      timeoutCounter = millis();
      getPackage();
    }
  }
  fin1.writeMicroseconds(finpwm[0]);
  fin2.writeMicroseconds(finpwm[1]);
  fin3.writeMicroseconds(finpwm[2]);
  fin4.writeMicroseconds(finpwm[3]);
  fin5.writeMicroseconds(finpwm[4]);
  fin6.writeMicroseconds(finpwm[5]);

  if(timeoutCounter + 100 < millis()){
    commLostFailsafe();
  }

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

  if(0 > servoNum && servoNum > 5){
    return;
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
        if(message.length() == 4){
          if (1000 < message.toInt() && message.toInt() < 2000) {
            finpwm[servoNum] = message.toInt();
          }
        }
        return;
      }
    }
  }
}

void commLostFailsafe(){
  while(true){
    int failsafeSet[6] = {1000, 1000, 1500, 1500, 1500, 1500};
    fin1.writeMicroseconds(failsafeSet[0]);
    fin2.writeMicroseconds(failsafeSet[1]);
    fin3.writeMicroseconds(failsafeSet[2]);
    fin4.writeMicroseconds(failsafeSet[3]);
    fin5.writeMicroseconds(failsafeSet[4]);
    fin6.writeMicroseconds(failsafeSet[5]);
  }
}