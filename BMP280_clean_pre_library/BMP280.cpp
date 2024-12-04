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
  SPI.transfer(0xF5 & _writeValue);
  SPI.transfer(_t_sb | _filter | _spi_mode);
  digitalWrite(_cs, HIGH);
  return 1;
}


void BMP280::update(){
  unsigned char* bytes = (unsigned char*)malloc(sizeof(unsigned char) * 6);
  digitalWrite(_cs, LOW);
  SPI.transfer(0xF7);
  read_continuous(bytes, 6);
  digitalWrite(_cs, HIGH);
  _rawPress = ((int32_t)*(bytes) << 12) | ((int32_t)*(bytes + 1) << 4) | ((int32_t)*(bytes + 2) >> 4);
  _rawTemp = ((int32_t)*(bytes + 3) << 12) | ((int32_t)*(bytes + 4) << 4) | ((int32_t)*(bytes + 5) >> 4);
  free(bytes);
  convertAll();
}


void BMP280::updateTemp(){
  // get raw temperature values
  unsigned char* bytes = (unsigned char*)malloc(sizeof(unsigned char) * 3);
  digitalWrite(_cs, LOW);
  SPI.transfer(0xFA);
  read_continuous(bytes, 3);
  digitalWrite(_cs, HIGH);
  _rawTemp = ((int32_t)*(bytes) << 12) | ((int32_t)*(bytes + 1) << 4) | ((int32_t)*(bytes + 2) >> 4);
  free(bytes);
  convertTemp();
}


void BMP280::updatePress(){
  // temperature is also needed to calculate pressure -> just use normal update function in that case
  update();
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

void BMP280::convertAll(){
  int32_t adc_T = _rawTemp;
  int32_t t_fine;
  int32_t tVar1, tVar2, T;

  tVar1 = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
  tVar2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

  t_fine = tVar1 + tVar2;
  T = (t_fine * 5 + 128) >> 8;

  _temp = T / 100.0;

  int32_t adc_P = _rawPress;
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_dig_P6;
  var2 = var2 + ((var1 * (int64_t)_dig_P5) << 17);
  var2 = var2 + (((int64_t)_dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_dig_P3) >> 8) + ((var1 * (int64_t)_dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_dig_P1) >> 33;
  if (var1 == 0)
  {
  _press = 0; // avoid exception caused by division by zero
  }
  else{
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_dig_P7) << 4);
    _press = p / 256.0;
  }
}

void BMP280::convertTemp(){
  int32_t t_fine;
  int32_t tVar1, tVar2, T;

  tVar1 = ((((_rawTemp >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;
  tVar2 = (((((_rawPress >> 4) - ((int32_t)_dig_T1)) * ((_rawPress >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

  t_fine = tVar1 + tVar2;
  T = (t_fine * 5 + 128) >> 8;

  _temp = T / 100.0;
}

void BMP280::getConstants(){
  digitalWrite(_cs, LOW);
  // get temperature constants
  unsigned char val1, val2;
  SPI.transfer(0x88 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_T1 = ((uint16_t)val1 << 8) | val2;
  //SPI.transfer(0x8A | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_T2 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x8C | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_T3 = ((int16_t)val1 << 8) | val2;
  // get pressure constants
  //SPI.transfer(0x8E | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P1 = ((uint16_t)val1 << 8) | val2;
  //SPI.transfer(0x90 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P2 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x92 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P3 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x94 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P4 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x96 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P5 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x98 | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P6 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x9A | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P7 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x9C | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P8 = ((int16_t)val1 << 8) | val2;
  //SPI.transfer(0x9E | _readValue);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  _dig_P9 = ((int16_t)val1 << 8) | val2;
  digitalWrite(_cs, HIGH);
}