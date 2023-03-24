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
#include <ArduinoJson.h>
#define Radius 55
#define NormalWaterL 62
#define AlertL 62
#define NoneL 154
#define SensorL 66
#define AcselQ 10
#define GyroQ 0.07
const char* ssid = "razdacha tacnev s tesakom";
const char* password =  "14888282";
const char* mqtt_server = "lapsoft.mooo.com";
const char* mqtt_login = "esp";
const char* mqtt_password = "L9{HTRfq7#N!";
const int mqtt_port = 10101;

void setupTopics();
#include <nto_view.h>
#line 63 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setup();
#line 69 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void loop();
#line 74 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void AutoLightTopic(String s);
#line 84 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void LightOnTopic(String s);
#line 97 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void FanTopic(String s);
#line 108 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void lightCheck();
#line 122 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void WindowTopic(String s);
#line 132 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void DoorTopic(String s);
#line 142 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void PumpTopic(String s);
#line 152 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void checkQuake();
#line 182 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void transfuse150();
#line 196 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void transfuse();
#line 209 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void onRobotCome();
#line 219 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void Alerts();
#line 224 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void checkWater();
#line 233 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getWaterByWT();
#line 236 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void TopicOut(String s);
#line 239 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void checkCO2TVOC();
#line 254 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_misc();
#line 263 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_water_color();
#line 268 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_water(float colb1_volume, float colb2_volume, float flowed_volume);
#line 273 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_alarm_quake();
#line 275 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_alarm_overflow();
#line 277 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_alarm_co2();
#line 279 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void send_alarm_tvoc();
#line 36 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
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
  //Alerts();
  delay(100);
  
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
  if (as>=AcselQ||gs>=GyroQ)
  {
    Serial.println("quake");
    client.publish("/lab/alarm/quake","1");
    client.publish("/lab/alarm/quake","1");
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
  Serial.println(as);
  Serial.println(gs);
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
  while (getWaterByWT()<AlertL*2*3.14*Radius&&IsRunPomp)
  {
    vTaskDelay(30);
  }
  pompOff();
  if (getWaterByWT()>=AlertL*2*3.14*Radius)
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
void Alerts(){
  checkWater();
  checkCO2TVOC();
  checkQuake();
}
void checkWater(){
  if (getWaterByWT()>AlertL*2*3.14*Radius)
  {
    client.publish("/lab/alarm/overflow","1");
    client.publish("/lab/alarm/overflow","1");
    client.publish("/lab/alarm/overflow","1");
    pompOff();
  }
}
float getWaterByWT(){
  return getWaterLVL()*SensorL/100*0.001*2*3.14*Radius;
}
void TopicOut(String s){
  Serial.println(s);
}
void checkCO2TVOC(){
  
  Wire.begin();
  CO30.measureAirQuality();
  if (CO30.CO2 > 600) {
    client.publish("/lab/alarm/co2","1");
    client.publish("/lab/alarm/co2","1");
    client.publish("/lab/alarm/co2","1");
  }
  if (CO30.TVOC > 150) {
    client.publish("/lab/alarm/tvoc","1");
    client.publish("/lab/alarm/tvoc","1");
    client.publish("/lab/alarm/tvoc","1");
  }
}
void send_misc(){  DynamicJsonDocument doc(512);
  doc["door_magnet_on"] = door;  doc["windows_magnet"] = window;
  doc["light_lux"] = getLux();  doc["co2_ppm"] = getCO2();
  doc["tvoc_ppm"] = getTVOC();  doc["pressure"] = getPressure();
  doc["temperature"] = getTemperature();  doc["humidity"] = getHumidity();
  doc["amperage"] = getToque();  doc["powerage"] = 12*getToque();
  String message;
  serializeJson(doc, message);  client.publish("/lab/misc", message.c_str());
}
void send_water_color(){
  DynamicJsonDocument doc(128);  String message;
  doc["color"] = getColor();
  serializeJson(doc, message);  Serial.println(client.publish("/lab/color", message.c_str()));
}
void send_water(float colb1_volume, float colb2_volume, float flowed_volume){  DynamicJsonDocument doc(256);
  doc["colb1_volume"] = colb1_volume;  doc["colb2_volume"] = colb2_volume;
  doc["flowed_volume"] = flowed_volume;  String message;
  serializeJson(doc, message);  client.publish("/lab/water", message.c_str());
  Serial.println(client.publish("/lab/water", message.c_str()));}
void send_alarm_quake(){
  client.publish("/lab/alarm/quake", 0);}
void send_alarm_overflow(){  client.publish("/lab/alarm/overflow", 0);
}
void send_alarm_co2(){  client.publish("/lab/alarm/co2", 0);
}
void send_alarm_tvoc(){
  client.publish("/lab/alarm/tvoc", 0);
}
