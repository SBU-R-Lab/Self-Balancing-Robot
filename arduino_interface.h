#ifndef _ARDUINO_INTERFACE_H
#define _ARDUINO_INTERFACE_H

#include <Arduino.h>
#include <Wire.h>

int i2cwrite(unsigned char slave_addr, unsigned char reg_addr, unsigned char len, unsigned char const *data);

int i2cread(unsigned char slave_addr, unsigned char reg_addr, unsigned char len, unsigned char *data);

void getmillis(unsigned long* count);

#endif
