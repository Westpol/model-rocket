#include "BMP280.h"

#define chipSelectPin 15


double temp;
double press;
double referencePress;

BMP280 bmp(1000000, chipSelectPin);

void setup() {
  Serial.begin(115200);
  while(!bmp.init()){
    Serial.println("No BMP280 Found!");
    delay(1000);
  }
  delay(1000);
  bmp.update();
  referencePress = bmp.getPress();
}

void loop() {

  bmp.update();
  temp = bmp.getTemp();
  press = bmp.getPress();

  Serial.println(temp);

  /*Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println("°C");
  Serial.print("  Pressure : ");
  Serial.print((press / 100.0));
  Serial.println("hpa");*/
  delay(100);
}