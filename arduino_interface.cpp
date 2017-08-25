#include "arduino_interface.h"

int i2cwrite(unsigned char slave_addr, unsigned char reg_addr, unsigned char len, unsigned char const *data)
{
  Wire.beginTransmission(slave_addr);
  Wire.write(reg_addr);
  Wire.write(data, len);
  Wire.endTransmission();
  return 0;
}

int i2cread(unsigned char slave_addr, unsigned char reg_addr, unsigned char len, unsigned char *data)
{
  Wire.beginTransmission(slave_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(slave_addr,len);
  for(int i = 0; i < len; i++)
    data[i] = Wire.read();
  
  return 0;
}

void getmillis(unsigned long* count)
{
  *count = millis();
}

