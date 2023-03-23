#include <Wire.h>
#include <iostream>
#include <I2C_graphical_LCD_display.h>
#include "Adafruit_APDS9960.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include "mcp3021.h"
#include <BH1750FVI.h>
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

#define ToquePort 26
#define DoorPort 15
#define WindowPort 23
#define DoorCheckPort 18
#define WindowCheckPort 19
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
Servo doorServo;
Servo windowServo;
PCA9536 pca9536;
I2C_graphical_LCD_display lcd;
uint16_t clear;
MCP3021 mcp3021;
SGP30 CO30;
Adafruit_APDS9960 apds9960;
BH1750FVI LightSensor_1;
Adafruit_BME280 bme280;
Adafruit_MPU6050 mpu;
VL53L0X lox;
#define ColorDistanceSensorAddr 0x07
#define WaterID 5
uint16_t ColorDistanceData[4];
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;
volatile double waterFlow=0; 
volatile bool door=0;
volatile bool window=0;
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
  doorServo.write(90);
}
void loop(){
  for (int i = 0; i < 1000; i++)
  {
    setLightLVL(i);
    delay(5);
  }
  
}
void StartAll(){
  std::cout<<"1"<<"\n";
  ledcSetup(0, 90000, 10);
  ledcAttachPin(4, 0);
  ledcAttachPin(13, 0);
  doorServo.attach(DoorPort);
  windowServo.attach(WindowPort);
  std::cout<<"2"<<"\n";
  Wire.begin();
  lcd.begin();
  std::cout<<"3"<<"\n";
  mpu.begin(0x69);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  std::cout<<"4"<<"\n";
  /*if(lox.init()){
    lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);
  }*/
  std::cout<<"5"<<"\n";
  Wire.begin();
  pca9536.reset();
  pca9536.setMode(IO_OUTPUT);
  pca9536.setState(IO0, IO_LOW);
  pca9536.setState(IO1, IO_LOW);
  pca9536.setState(IO2, IO_LOW);
  pca9536.setState(IO3, IO_LOW);
  analogReadResolution(12);
  mcp3021.begin(WaterID);
  CO30.begin();
  CO30.initAirQuality();
  std::cout<<"6"<<"\n";
  LightSensor_1.begin();
  bme280.begin();
  LightSensor_1.setMode(Continuously_High_Resolution_Mode);
  std::cout<<"7"<<"\n";
  pinMode(5, INPUT_PULLDOWN);
  attachInterrupt(5, waterFlowISR, RISING);
  pinMode(DoorCheckPort, INPUT_PULLDOWN);
  pinMode(WindowCheckPort, INPUT_PULLDOWN);
  attachInterrupt(DoorCheckPort, DoorISR, CHANGE);
  attachInterrupt(WindowCheckPort, WindowISR, CHANGE);
  std::cout<<"8"<<"\n";
  Serial.begin(115200);
  //WifiConnect();
  xTaskCreate(MQTTClientTask,"MQTTTask",10*1024,NULL,1,NULL);
}


/* Sensors */
void setLightLVL(uint16_t LVL){
  ledcWrite(0, LVL);
}
void pompOn(){
  pca9536.setState(IO2, IO_HIGH);
}
void pompOff(){
  pca9536.setState(IO2, IO_LOW);
}
void coolerOn(){
  pca9536.setState(IO1, IO_HIGH);
}
void coolerOff(){
  pca9536.setState(IO1, IO_LOW);
}
void DoorISR(){
  delay(1);
  if(digitalRead(DoorCheckPort)==HIGH){
    door=true;
  }
  else{
    door=false;
  }
}
void WindowISR(){
  delay(1);
  if(digitalRead(WindowCheckPort)==HIGH){
    window=true;
  }
  else{
    window=false;
  }
}
void openDoor(){
  if (!door)
  {
    doorServo.write(90);
  }
}
void closeDoor(){
  if (door)
  {
    doorServo.write(0);
  }
  
}
void openWindow(){
  if (!window)
  {
    windowServo.write(90);
  }
  
}
void closeWindow(){
  if (window)
  {
    windowServo.write(0);
  }
}
void waterFlowISR(){
  waterFlow += 1.0 / 5880.0;
}
void lcdPrint(String s){
  lcd.string( s.c_str(),false);
}

void lcdClear(){
  lcd.clear (0, 0, 128, 64, 0x00);
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
sensors_event_t getGyroscope(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  return g;
}
float getToque(){
  float sensorValue=0;
  for (int i = 0; i < 50; i++)
  {
    sensorValue += analogRead(ToquePort);
    delay(2);
  }
  return (sensorValue/50*3.3/4096-2.5)/ 0.185;
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

