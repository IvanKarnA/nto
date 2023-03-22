#include <Arduino.h>
#line 1 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
#include <Wire.h>
#include <iostream>
#include "Adafruit_APDS9960.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include "mcp3021.h"
#include <BH1750FVI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <I2C_graphical_LCD_display.h>
#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08
#define GP16 0x07
#define GP4 0x06
#define GP14 0x05
#define GP5 0x04
#define GP18 0x03
I2C_graphical_LCD_display lcd;
uint16_t clear;
MCP3021 mcp3021;
SGP30 CO30;
Adafruit_APDS9960 apds9960;
BH1750FVI LightSensor_1;
Adafruit_BME280 bme280;
#define ColorDistanceSensorAddr 0x07
#define WaterID 5
uint16_t ColorDistanceData[4];
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;
/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/

#line 42 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setup();
#line 45 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void loop();
#line 49 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void StartAll();
#line 59 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getTemperature();
#line 62 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getHumidity();
#line 65 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getPressure();
#line 68 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
uint16_t getCO2();
#line 73 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
uint16_t getTVOC();
#line 78 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
uint16_t getLux();
#line 81 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
bool setBusChannel(uint8_t i2c_channel);
#line 96 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
int getWaterLVL();
#line 101 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
bool ColorDistanceSensorBegin();
#line 112 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void ColorDistanceGetData();
#line 42 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setup(){
  StartAll();
}
void loop(){
  std::cout<< getTemperature()<<"  "<< getHumidity()<<"  "<< getPressure()<<"\n";
  delay(100);
}
void StartAll(){
  Wire.begin();
  mcp3021.begin(WaterID);
 CO30.begin();
CO30.initAirQuality();
LightSensor_1.begin();
bme280.begin();
LightSensor_1.setMode(Continuously_High_Resolution_Mode);
}

float getTemperature(){
  return bme280.readTemperature();
}
float getHumidity(){
  return bme280.readHumidity();
}
float getPressure(){
  return bme280.readPressure();
}
uint16_t getCO2(){
  Wire.begin();
  CO30.measureAirQuality();
  return CO30.CO2;
}
uint16_t getTVOC(){
  Wire.begin();
  CO30.measureAirQuality();
  return CO30.CO2;
}
uint16_t getLux(){
  return LightSensor_1.getAmbientLight();
}
bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);
    Wire.endTransmission();
    return true;
    
  }
}
int getWaterLVL(){
  int x=map(mcp3021.readADC(), air_value, water_value, moisture_0, moisture_100);
  std::cout<<"Water lvl: "<<x<<"\n";
  return x;
}
bool ColorDistanceSensorBegin(){
  if (apds9960.begin())
    {
      apds9960.enableColor(true);
      apds9960.enableProximity(true);
      return true;
    }
    else{
      return false;
    }
}
void ColorDistanceGetData(){
  setBusChannel(ColorDistanceSensorAddr);
  while (!apds9960.colorDataReady())
  {
    delay(5);
  }
  
  apds9960.getColorData(&ColorDistanceData[0], &ColorDistanceData[1], &ColorDistanceData[2], &clear);
  ColorDistanceData[3] =apds9960.readProximity();
}


