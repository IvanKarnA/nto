#include <Wire.h>
#include <iostream>
#include "Adafruit_APDS9960.h"
#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08
#define GP16 0x07
#define GP4 0x06
#define GP14 0x05
#define GP5 0x04
#define GP18 0x03
SemaphoreHandle_t availableMultiplexor;
Adafruit_APDS9960 apds9960;
#define ColorDistanceSensorAddr GP16


void setup() {
  availableMultiplexor = xSemaphoreCreateBinary();

}

void loop() {
  

}

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/
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
namespace ColorDistanceSensor
{
  SemaphoreHandle_t availableToRead;
  SemaphoreHandle_t availableToWrite;
  bool begin(){
    availableToRead=xSemaphoreCreateBinary();
    availableToWrite=xSemaphoreCreateBinary();
    xSemaphoreGive(availableToWrite);
    return apds9960.begin();
    apds9960.enableColor(true);
    apds9960.enableProximity(true);
  }
  uint16_t Data[4];
  void DataScaner(void* pvParameters){
    if (!begin())
    {
      std::cout<< "ColorDistanceSensor: init error";
      while (1)
      {
        /* code */
      }
      
    }
    else
    {
      apds9960.enableColor(true);
      apds9960.enableProximity(true);
      for (;;)
      {
        while (!apds9960.colorDataReady()) {
        vTaskDelay(10);
      }
        if (xSemaphoreTake(availableToWrite,portMAX_DELAY)==pdPASS)
        {
          if (xSemaphoreTake(availableMultiplexor,portMAX_DELAY)==pdPASS)
          {
            setBusChannel(ColorDistanceSensorAddr);
            uint16_t clear;
            apds9960.getColorData(&Data[0], &Data[1], &Data[2], &clear);
            Data[3] =apds9960.readProximity();
            xSemaphoreGive(availableToRead);
            uint16_t clear;
            apds9960.getColorData(&Data[0], &Data[1], &Data[2], &clear);
            Data[3] =apds9960.readProximity();
            xSemaphoreGive(availableToRead);
            xSemaphoreGive(availableMultiplexor);
          }
          
        }
        
      }
      
    }
    
    
  }
} 
