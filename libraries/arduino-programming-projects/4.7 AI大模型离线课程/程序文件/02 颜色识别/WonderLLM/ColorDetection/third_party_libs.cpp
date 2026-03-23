// 将第三方库源文件聚合进编译，位于工程根目录以确保被 Arduino 构建系统编译。

#include "lib/adafruit/Adafruit_BusIO_Register.cpp"
#include "lib/adafruit/Adafruit_GenericDevice.cpp"
#include "lib/adafruit/Adafruit_GFX.cpp"
#include "lib/adafruit/Adafruit_I2CDevice.cpp"
#include "lib/adafruit/Adafruit_SPIDevice.cpp"
#include "lib/adafruit/Adafruit_SPITFT.cpp"
#include "lib/adafruit/Adafruit_ST7789.cpp"
#include "lib/adafruit/Adafruit_ST77xx.cpp"
#include "lib/adafruit/glcdfont.c"
