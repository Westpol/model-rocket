#include "SPI.h"
#include "BMP280.h"

#define chipSelectPin 10


double temp;
double press;

BMP280 bmp(1000000, chipSelectPin);

void setup() {
  Serial.begin(115200);
  while(!bmp.init()){
    Serial.println("No BMP280 Found!");
    delay(1000);
  }
  delay(100);
}

void loop() {
  bmp.update();
  temp = bmp.getTemp();
  press = bmp.getPress();
  Serial.print("Temp raw val:  ");
  Serial.println(temp, DEC);
  Serial.print("Press raw val: ");
  Serial.println(press, DEC);
  delay(100);
}