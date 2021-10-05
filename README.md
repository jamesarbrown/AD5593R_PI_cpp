# AD5593R C++ Library for Raspberry Pi
This is a C++ Library for AD5593R on Raspberry Pi, ported directly from the library for [Arduino](https://github.com/LukasJanavicius/AD5593R-Arduino-ESP32-Library)  
Based on a simmilar project for the Si5351 by [wwanc](https://github.com/wwanc/Si5351Pi_cpp)  
This project is compatable with the [ADAC CLICK](https://www.mikroe.com/adac-click-click)  
All interfaces are the same as those in the Arduino library except for the constructor, which takes an extra parameter for the I2C device path, see below. Refer to the Arduino project for the complete reference of the interfaces.
```
AD5593R ad5593r("/dev/i2c-1"); //Default i2c address AD5593R_BUS_BASE_ADDR (0x10)
```
Or
```
AD5593R ad5593r("/dev/i2c-1", AD5593R_BUS_BASE_ADDR, a0); // Replace AD5593R_BUS_BASE_ADDR with correct address if it is not 0x10
                                                          // a0 is the pin to switch address, in case multiple chips are in use
```

A custom function "init()" has also been added, this is just so we can see if the i2c connection was opened successfully.

Compiled using:
```
g++ -o main main.cpp AD5593R.cpp -lwiringPi
```
