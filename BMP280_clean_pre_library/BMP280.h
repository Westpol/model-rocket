#ifndef BMP280_h

#define BMP280_h


#include "Arduino.h"

class BMP280{
  public:
  BMP280(unsigned long speed, int cs);
  int init();
  void update();
  double getTemp();
  double getPress();

  private:
  unsigned long _spi_speed;
  int _cs;

  unsigned char _readValue = 0x80;
  unsigned char _writeValue = 0x7F;

  // Mode setup normal, 16x press, 16x temp
  unsigned char _mode = B00000011;  // Normal Mode [1,1]
  unsigned char _osrs_p = B00010100; // oversampling x16 [1, 0, 1]
  unsigned char _osrs_t = B01000000; // oversampling x16 [1, 0, 1]
  
  unsigned char _t_sb = B00000000;
  unsigned char _filter = B00011100;
  unsigned char _spi_mode = B00000000;

  double _temp = 0;
  double _press = 0;
  int32_t _rawTemp = 0;
  int32_t _rawPress = 0;

  // constant calibration values temperature
  uint16_t _dig_T1;
  int16_t _dig_T2;
  int16_t _dig_T3;

  // constant calibration values pressure
  uint16_t _dig_P1;
  int16_t _dig_P2;
  int16_t _dig_P3;
  int16_t _dig_P4;
  int16_t _dig_P5;
  int16_t _dig_P6;
  int16_t _dig_P7;
  int16_t _dig_P8;
  int16_t _dig_P9;

  void read_continuous(unsigned char* bytes, int len);
  void getConstants();
  void convertAll();
  void convertTemp();
};


#define T1_sample 27504;
#define T2_sample 26435;
#define T3_sample -1000;
#define P1_sample 36477;
#define P2_sample -10685;
#define P3_sample 3024;
#define P4_sample 2855;
#define P5_sample 140;
#define P6_sample -7;
#define P7_sample 15500;
#define P8_sample -14600;
#define P9_sample 6000;
#define T_sample 519888;
#define P_sample 415148;
#endif