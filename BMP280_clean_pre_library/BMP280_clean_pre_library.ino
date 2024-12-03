#include <SPI.h>

struct BMP280{
  uint32_t (*get_raw_temp);
  uint32_t (*get_raw_press);
};

typedef struct BMP280 BMP280;

#define chipSelectPin 10
unsigned char readValue = 0x80;
unsigned char writeValue = 0x7F;

// Mode setup normal, 16x press, 16x temp
unsigned char mode = B00000011;  // Normal Mode [1,1]
unsigned char osrs_p = B00010100; // oversampling x16 [1, 0, 1]
unsigned char osrs_t = B10100000; // oversampling x16 [1, 0, 1]
uint32_t temp;
uint32_t press;


void setup() {
  BMP280 bmp;
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  pinMode(chipSelectPin, OUTPUT);
  delay(100);
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(0xF4 & writeValue);
  SPI.transfer(mode | osrs_p | osrs_t);

  digitalWrite(chipSelectPin, HIGH);
}

void loop() {

  int len = 6;
  unsigned char* bytes = malloc(sizeof(unsigned char) * len);
  unsigned char reg = 0xF7;

  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(reg);

  read_continuous(bytes, len);

  digitalWrite(chipSelectPin, HIGH);

  press = ((uint32_t)*(bytes) << 12) + ((uint32_t)*(bytes + 1) << 4) + ((uint32_t)*(bytes + 2) >> 4);
  temp = ((uint32_t)*(bytes + 3) << 12) + ((uint32_t)*(bytes + 4) << 4) + ((uint32_t)*(bytes + 5) >> 4);

  Serial.print("Temp raw val:  ");
  Serial.println(temp, DEC);
  Serial.print("Press raw val: ");
  Serial.println(press, DEC);
  //Serial.println(temp, BIN);
  //Serial.println(temp, BIN);

  //printBits(bytes, len);
  free(bytes);
  delay(100);
}

int read_continuous(unsigned char* bytes, int len){
  for(int i = 0; i < len; i++){
    *(bytes + i) = SPI.transfer(0);
  }
}

int printBits(unsigned char* bytes, int len){
  for(int i = 0; i < len; i++){
    for(int8_t aBit = 7; aBit >= 0; aBit--){
      Serial.write(bitRead(*(bytes + i), aBit) ? '1' : '0');
    }
  }
}

uint32_t get_raw_temp(){
  unsigned char* bytes = malloc(sizeof(unsigned char) * 3);
  SPI.transfer(0xFA);
  read_continuous(bytes, 3);
  uint32_t value = ((uint32_t)*(bytes) << 12) + ((uint32_t)*(bytes + 1) << 4) + ((uint32_t)*(bytes + 2) >> 4);
  free(bytes);
  return value;
}

uint32_t get_raw_press(){
  unsigned char* bytes = malloc(sizeof(unsigned char) * 3);
  SPI.transfer(0xF7);
  read_continuous(bytes, 3);
  uint32_t value = ((uint32_t)*(bytes) << 12) + ((uint32_t)*(bytes + 1) << 4) + ((uint32_t)*(bytes + 2) >> 4);
  free(bytes);
  return value;
}