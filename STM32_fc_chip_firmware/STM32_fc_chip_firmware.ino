HardwareSerial Serial_p_4(PC11, PC10);

void setup() {
  Serial_p_4.begin(115200);

}

void loop() {
  Serial_p_4.print("Hello, World!\n");
  delay(500);

}
