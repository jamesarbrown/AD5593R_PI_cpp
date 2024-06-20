# AD5593R C++ Library for Raspberry Pi

## Overview
The ADE5593R is a 8 Channel configurable DAC/ADC/GPO device using i2c communications.
This is a C++ Library for usage on a Raspberry Pi. 

## Useage
Please see main.ccp as examples of calling the library functions

The constructor shown in main.ccp can take an extra parameter for the I2C device path, see below.
```
AD5593R ad5593r("/dev/i2c-1"); //Default i2c address AD5593R_BUS_BASE_ADDR (0x10)
```
Or
```
AD5593R ad5593r("/dev/i2c-1", AD5593R_BUS_BASE_ADDR, a0); // Replace AD5593R_BUS_BASE_ADDR with correct address if it is not 0x10
                                                          // a0 is the pin to switch address, in case multiple chips are in use
```

Note on the CJMCU the A0 pin is pulled high, requring a 0x11 address as standard, or pull it to GND with a link for default address
```
AD5593R ad5593r("/dev/i2c-1", 0x11);
```
### Compile
```
g++ -o main main.cpp AD5593R.cpp -lwiringPi
```
## Devices
The AD5593 is usually found in a TSSOP package and of course can be direct onto your board. If you are not using a board and want to look at a wired solution, or even perhaps avoid soldering on such a tiny level, there are pre-prepared boards.

[ADAC CLICK](https://www.mikroe.com/adac-click-click)

[CJMCU / Purple Aliexpress](https://www.aliexpress.com/w/wholesale-ad5593.html?spm=a2g0o.productlist.search.0)

This library was tested on a CJMCU.

## Chip Instructions
[Analog Devices AD5593R](https://www.analog.com/media/en/technical-documentation/data-sheets/ad5593r.pdf)

## History
Originally ported directly from the library for [Arduino](https://github.com/LukasJanavicius/AD5593R-Arduino-ESP32-Library) by @zzzbatmand Sir Wintour

Based on a simmilar project for the Si5351 by [wwanc](https://github.com/wwanc/Si5351Pi_cpp)  



