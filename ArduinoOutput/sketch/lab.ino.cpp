#include <Arduino.h>
#line 1 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
#include <Wire.h>
#include <iostream>
#include <I2C_graphical_LCD_display.h>
#include "Adafruit_APDS9960.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include "mcp3021.h"
#include <BH1750FVI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include "WiFi.h"
#include <PubSubClient.h>
#include <map>
#include <string.h>
#include <VL53L0X.h>

#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08
#define GP16 0x07
#define GP4 0x06
#define GP14 0x05
#define GP5 0x04
#define GP18 0x03
typedef void (*callbackScript)(String);
std::map<String,callbackScript> topics;
VL53L0X lox;
I2C_graphical_LCD_display lcd;
uint16_t clear;
MCP3021 mcp3021;
SGP30 CO30;
Adafruit_APDS9960 apds9960;
BH1750FVI LightSensor_1;
Adafruit_BME280 bme280;
MPU6050 mpu;
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

const char* ssid = "****";
const char* password =  "****";
const char* mqtt_server = "lapsoft.mooo.com";
const char* mqtt_login = "esp";
const char* mqtt_password = "L9{HTRfq7#N!";
const int mqtt_port = 10101;

WiFiClient espClient;
PubSubClient client(espClient);

/* MQTT */
//Функция для оформления подписки
#line 65 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void subscribe(const char* name, callbackScript script);
#line 71 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void MQTTcallback(char* topic, byte* payload, unsigned int length);
#line 88 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setupTopics();
#line 92 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void TopicOut(String s);
#line 97 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setup();
#line 100 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void loop();
#line 104 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void StartAll();
#line 125 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void lcdPrint(String s);
#line 128 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getTemperature();
#line 131 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getHumidity();
#line 134 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getPressure();
#line 137 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
uint16_t getCO2();
#line 142 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
uint16_t getTVOC();
#line 147 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
uint16_t getLux();
#line 150 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
bool setBusChannel(uint8_t i2c_channel);
#line 165 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
int getWaterLVL();
#line 170 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
bool ColorDistanceSensorBegin();
#line 181 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void ColorDistanceGetData();
#line 191 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
float getDistanceLaser();
#line 194 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
Vector getGyroscope();
#line 199 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void MQTTClientTask(void* pvParameters);
#line 238 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void WifiConnect();
#line 65 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void subscribe(const char* name, callbackScript script){
  topics.insert(std::make_pair(String(name),script));
  client.subscribe(name);
}

//Вызывается при получении сообщения с топиков
void MQTTcallback(char* topic, byte* payload, unsigned int length) 
{
  String message;
  for (int i = 0; i < length; i++) 
  {
    message = message + (char)payload[i];
  }
  
  if ((topics.find(String(topic)))==topics.end())
  {
    Serial.println("Error");
  }
  else{
    ((*topics.find(String(topic))).second)(message);
  }
}

void setupTopics(){
    // subscribe("esp/test", TopicOut);
}

void TopicOut(String s){
  Serial.println(s);
}


void setup(){
  StartAll();
}
void loop(){
  
  delay(100);
}
void StartAll(){
  Wire.begin();
  lcd.begin();
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  mpu.setThreshold(3);
  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
  mcp3021.begin(WaterID);
  CO30.begin();
  CO30.initAirQuality();
  LightSensor_1.begin();
  bme280.begin();
  LightSensor_1.setMode(Continuously_High_Resolution_Mode);
  Serial.begin(115200);
  WifiConnect();
  xTaskCreate(MQTTClientTask,"MQTTTask",10*1024,NULL,1,NULL);
}


/* Sensors */
void lcdPrint(String s){
  lcd.string( s.c_str(),false);
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
float getDistanceLaser(){
  return lox.readRangeSingleMillimeters();
}
Vector getGyroscope(){
  return mpu.readNormalizeGyro();
}

/* MQTT */
void MQTTClientTask(void* pvParameters){
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(MQTTcallback);
  while (!client.connected()) 
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("esp", mqtt_login,mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      vTaskDelay(1000);
    }
  }
  setupTopics();
  while (1)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WIFI not connected");
        vTaskDelay(3000);
    }
    else{
        if (client.connected())
        {
            client.loop();
            vTaskDelay(100);
        }
        else{
            Serial.println("MQTT not connected");
            vTaskDelay(1000);
        }
    } 
  } 
}

void WifiConnect(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.SSID());
}


