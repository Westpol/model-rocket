#include <Servo.h>

Servo myservo;

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(A0, INPUT);
  Serial.begin(115200);
}

void loop() {
  myservo.writeMicroseconds(map(analogRead(A0),0,1024,1000,2000));
  Serial.println(analogRead(A0));
}
