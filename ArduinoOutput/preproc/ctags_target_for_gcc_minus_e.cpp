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

# 20 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
#define Radius 55
#define NormalWaterL 62
#define AlertL 62
#define NoneL 154
#define SensorL 66
#define AcselQ 10
#define GyroQ 0.07
const char* ssid = "razdacha tacnev s tesakom";
const char* password = "14888282";
const char* mqtt_server = "lapsoft.mooo.com";
const char* mqtt_login = "esp";
const char* mqtt_password = "L9{HTRfq7#N!";
const int mqtt_port = 10101;

void setupTopics();
# 36 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
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
# 52 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
bool AutoLight=false;



/* MQTT */
//Функция для оформления подписки





void setup(){
  WifiConnect();
  StartAll();


}
void loop(){
  Alerts();
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
  if (as>=10||gs>=0.07)
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
  while (waterFlow<=150&&getWaterByWT()<62*2*3.14*55)
  {
    delay(2);
  }
  pompOff();
  if (getWaterByWT()>62*2*3.14*55)
  {
    client.publish("/lab/alarm/overflow","1");
  }
}

void transfuse(){
  waterFlow=0;
  pompOn();
  while (getWaterByWT()<62*2*3.14*55&&IsRunPomp)
  {
    vTaskDelay(30);
  }
  pompOff();
  if (getWaterByWT()>=62*2*3.14*55)
  {
    client.publish("/lab/alarm/overflow","1");
  }
}
void onRobotCome(){
  uint16_t x=0;
  while (apds9960.readProximity()>154)
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
  if (getWaterByWT()>62*2*3.14*55)
  {
    client.publish("/lab/alarm/overflow","1");
    client.publish("/lab/alarm/overflow","1");
    client.publish("/lab/alarm/overflow","1");
    pompOff();
  }
}
float getWaterByWT(){
  return getWaterLVL()*66/100*0.001*2*3.14*55;
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
void send_misc(){ DynamicJsonDocument doc(512);
  doc["door_magnet_on"] = door; doc["windows_magnet"] = window;
  doc["light_lux"] = getLux(); doc["co2_ppm"] = getCO2();
  doc["tvoc_ppm"] = getTVOC(); doc["pressure"] = getPressure();
  doc["temperature"] = getTemperature(); doc["humidity"] = getHumidity();
  doc["amperage"] = getToque(); doc["powerage"] = 12*getToque();
  String message;
  serializeJson(doc, message); client.publish("/lab/misc", message.c_str());
}
void send_water_color(){
  DynamicJsonDocument doc(128); String message;
  doc["color"] = getColor();
  serializeJson(doc, message); Serial.println(client.publish("/lab/color", message.c_str()));
}
void send_water(float colb1_volume, float colb2_volume, float flowed_volume){ DynamicJsonDocument doc(256);
  doc["colb1_volume"] = colb1_volume; doc["colb2_volume"] = colb2_volume;
  doc["flowed_volume"] = flowed_volume; String message;
  serializeJson(doc, message); client.publish("/lab/water", message.c_str());
  Serial.println(client.publish("/lab/water", message.c_str()));}
void send_alarm_quake(){
  client.publish("/lab/alarm/quake", 0);}
void send_alarm_overflow(){ client.publish("/lab/alarm/overflow", 0);
}
void send_alarm_co2(){ client.publish("/lab/alarm/co2", 0);
}
void send_alarm_tvoc(){
  client.publish("/lab/alarm/tvoc", 0);
}
