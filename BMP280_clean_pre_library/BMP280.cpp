#include "BMP280.h"
#include "Arduino.h"
#include "SPI.h"


BMP280::BMP280(unsigned long spi_speed, int cs){
  _spi_speed = spi_speed;
  _cs = cs;
}


int BMP280::init(){
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(_spi_speed, MSBFIRST, SPI_MODE0));

  digitalWrite(_cs, LOW);   // check if chip has the correct ID
  SPI.transfer(0xD0 | _readValue);
  unsigned char id = SPI.transfer(0);
  digitalWrite(_cs, HIGH);
  if(id != 0x58){
    return 0;
  }

  getConstants();

  digitalWrite(_cs, LOW);   // write mode and settings to chip
  SPI.transfer(0xF4 & _writeValue);
  SPI.transfer(_mode | _osrs_p | _osrs_t);
  digitalWrite(_cs, HIGH);
  return 1;
}


void BMP280::update(){
  unsigned char* bytes = (unsigned char*)malloc(sizeof(unsigned char) * 6);
  SPI.transfer(0xF7);
  read_continuous(bytes, 6);
  _press = ((uint32_t)*(bytes) << 12) + ((uint32_t)*(bytes + 1) << 4) + ((uint32_t)*(bytes + 2) >> 4);
  _temp = ((uint32_t)*(bytes + 3) << 12) + ((uint32_t)*(bytes + 4) << 4) + ((uint32_t)*(bytes + 5) >> 4);
  free(bytes);
  convertData();
}


void BMP280::updateTemp(){
  // get raw temperature values
  unsigned char* bytes = (unsigned char*)malloc(sizeof(unsigned char) * 3);
  SPI.transfer(0xFA);
  read_continuous(bytes, 3);
  _temp = ((uint32_t)*(bytes) << 12) + ((uint32_t)*(bytes + 1) << 4) + ((uint32_t)*(bytes + 2) >> 4);
  free(bytes);
  convertData();
}


void BMP280::updatePress(){
  // get raw pressure values
  unsigned char* bytes = (unsigned char*)malloc(sizeof(unsigned char) * 3);
  SPI.transfer(0xF7);
  read_continuous(bytes, 3);
  _press = ((uint32_t)*(bytes) << 12) + ((uint32_t)*(bytes + 1) << 4) + ((uint32_t)*(bytes + 2) >> 4);
  free(bytes);
  convertData();
}


double BMP280::getTemp(){
  return _temp;
}


double BMP280::getPress(){
  return _press;
}


void BMP280::read_continuous(unsigned char* bytes, int len){
  for(int i = 0; i < len; i++){
    *(bytes + i) = SPI.transfer(0);
  }
}

void BMP280::convertData(){
  int32_t t_fine;
  int32_t var1, var2, T;

  var1 = ((((_rawTemp >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
  var2 = (((((_rawPress >> 4) - ((int32_t)_dig_T1)) * ((_rawPress >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;

  _temp = T / 100.0;
}

void BMP280::getConstants(){

  // get temperature constants
  unsigned char val1, val2;
  SPI.transfer(0x88 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_T1 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x8A | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_T2 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x8C | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_T3 = ((uint16_t)val1 << 8) + val2;


  // get pressure constants
  SPI.transfer(0x8E | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P1 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x90 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P2 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x92 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P3 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x94 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P4 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x96 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P5 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x98 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P6 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x9A | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P7 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x9C | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P8 = ((uint16_t)val1 << 8) + val2;
  SPI.transfer(0x9E | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P9 = ((uint16_t)val1 << 8) + val2;
}