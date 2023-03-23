#include <Arduino.h>
#line 1 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
#include <Wire.h>
#include <iostream>
#include <I2C_graphical_LCD_display.h>
#include "Adafruit_APDS9960.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include "mcp3021.h"
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <VL53L0X.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <map>
#include <string.h>
#include <ESP32Servo.h>
#include "PCA9536.h"
#include <math.h>
#define Radius 1
#define NormalWaterL 40
#define AlertL 170
#define NoneL 100
#define SensorL 150
#define AcselQ 12
#define GyroQ 12
const char* ssid = "****";
const char* password =  "****";
const char* mqtt_server = "lapsoft.mooo.com";
const char* mqtt_login = "esp";
const char* mqtt_password = "L9{HTRfq7#N!";
const int mqtt_port = 10101;

void setupTopics();
#include <nto_view.h>
#line 62 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setup();
#line 66 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void loop();
#line 71 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void AutoLightTopic(String s);
#line 81 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void LightOnTopic(String s);
#line 94 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void FanTopic(String s);
#line 105 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void lightCheck();
#line 119 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void WindowTopic(String s);
#line 129 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void DoorTopic(String s);
#line 139 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void PumpTopic(String s);
#line 149 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void checkQuake();
#line 173 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void transfuse150();
#line 186 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void transfuse();
#line 199 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void onRobotCome();
#line 209 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getWaterByWT();
#line 212 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void TopicOut(String s);
#line 215 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void checkCO2TVOC();
#line 35 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setupTopics(){
    // subscribe("esp/test", TopicOut);
    subscribe("/lab/ctl/pump",PumpTopic);
    subscribe("/lab/ctl/door",DoorTopic);
    subscribe("/lab/ctl/window",WindowTopic);
    subscribe("/lab/ctl/light",LightOnTopic);
    subscribe("/lab/ctl/auto-lighting",AutoLightTopic);
    subscribe("/lab/ctl/fan",FanTopic);
}
/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/
bool AutoLight=false;



/* MQTT */
//Функция для оформления подписки





void setup(){
  StartAll();
  
}
void loop(){
  
  
  
}
void AutoLightTopic(String s){
  if (s=="true")
  {
    AutoLight=true;
  }
  else
  {
    AutoLight=false;
  }
}
void LightOnTopic(String s){
  if (!AutoLight)
  {
    if (s=="true")
  {
    setLightLVL(1023);
  }
  else
  {
    setLightLVL(0);
  }
  }
}
void FanTopic(String s){
if (s=="true")
  {
    coolerOn();
  }
  else
  {
    coolerOn();
  }
}

void lightCheck(){
  if (AutoLight)
  {

    int x=200-getLux();
    if (x>0)
    {
      setLightLVL(x*5);
    }
    else{
      setLightLVL(1024+x*5);
    }
  }
}
void WindowTopic(String s){
  if (s=="true")
  {
    openWindow();
  }
  else
  {
    closeWindow();
  }
}
void DoorTopic(String s){
  if (s=="true")
  {
    openDoor();
  }
  else
  {
    closeDoor();
  }
}
void PumpTopic(String s){
  if (s=="true")
  {
    transfuse();
  }
  else
  {
    pompOff();
  }
}
void checkQuake(){
  sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  float as=sqrtf(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y+a.acceleration.x*a.acceleration.z),gs=sqrtf(g.gyro.x*g.gyro.x+g.gyro.y*g.gyro.y+g.gyro.z*g.gyro.z);
  if (as>AcselQ||gs>GyroQ)
  {
    client.publish("/lab/alarm/quake","1");
  }
    Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
}
void transfuse150(){
  waterFlow=0;
  pompOn();
  while (waterFlow<=150&&getWaterByWT()<AlertL*2*3.14*Radius)
  {
    delay(2);
  }
  pompOff();
  if (getWaterByWT()>AlertL*2*3.14*Radius)
  {
    client.publish("/lab/alarm/overflow","1");
  }
}
void transfuse(){
  waterFlow=0;
  pompOn();
  while (getWaterByWT()<AlertL*2*3.14*Radius)
  {
    delay(2);
  }
  pompOff();
  if (getWaterByWT()>AlertL*2*3.14*Radius)
  {
    client.publish("/lab/alarm/overflow","1");
  }
}
void onRobotCome(){
  uint16_t x=0;
  while (apds9960.readProximity()>NoneL)
  {
    vTaskDelay(100);
    
  }
  vTaskDelay(2000);
  
}
float getWaterByWT(){
  return getWaterLVL()*SensorL/100*0.000001*2*3.14*Radius;
}
void TopicOut(String s){
  Serial.println(s);
}
void checkCO2TVOC(){
  Wire.begin();
  CO30.measureAirQuality();
  if (CO30.CO2 > 600) {
    client.publish("/lab/alarm/co2","1");
  }
  if (CO30.TVOC > 150) {
    client.publish("/lab/alarm/tvoc","1");
  }
}
