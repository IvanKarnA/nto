# 1 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
# 2 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 3 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 4 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 5 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 6 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 7 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 8 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 9 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 10 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 11 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 12 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 13 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 14 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 15 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 16 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 17 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 18 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2

#define Radius 1
#define NormalWaterL 40
#define AlertL 170
#define NoneL 100
#define SensorL 150
#define AcselQ 12
#define GyroQ 12
const char* ssid = "****";
const char* password = "****";
const char* mqtt_server = "lapsoft.mooo.com";
const char* mqtt_login = "esp";
const char* mqtt_password = "L9{HTRfq7#N!";
const int mqtt_port = 10101;

void setupTopics();
# 35 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
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
# 51 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
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
  if (as>12||gs>12)
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
  while (waterFlow<=150&&getWaterByWT()<170*2*3.14*1)
  {
    delay(2);
  }
  pompOff();
  if (getWaterByWT()>170*2*3.14*1)
  {
    client.publish("/lab/alarm/overflow","1");
  }
}
void transfuse(){
  waterFlow=0;
  pompOn();
  while (getWaterByWT()<170*2*3.14*1)
  {
    delay(2);
  }
  pompOff();
  if (getWaterByWT()>170*2*3.14*1)
  {
    client.publish("/lab/alarm/overflow","1");
  }
}
void onRobotCome(){
  uint16_t x=0;
  while (apds9960.readProximity()>100)
  {
    vTaskDelay(100);

  }
  vTaskDelay(2000);

}
float getWaterByWT(){
  return getWaterLVL()*150/100*0.000001*2*3.14*1;
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
