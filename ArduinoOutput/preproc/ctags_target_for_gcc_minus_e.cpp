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
#define I2C_HUB_ADDR 0x70
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08
#define GP16 0x07
#define GP4 0x06
#define GP14 0x05
#define GP5 0x04
#define GP18 0x03
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
# 40 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
void setup(){
  StartAll();
}
void loop(){
  std::cout<< getTemperature()<<"  "<< getHumidity()<<"  "<< getPressure()<<"\n";
  delay(100);
}
void StartAll(){
  Wire.begin();
  mcp3021.begin(5);
 CO30.begin();
CO30.initAirQuality();
LightSensor_1.begin();
bme280.begin();
LightSensor_1.setMode(0x10);
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
  if (i2c_channel >= 0x08)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(0x70);
    Wire.write(i2c_channel | 0x08);
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
  setBusChannel(0x07);
  while (!apds9960.colorDataReady())
  {
    delay(5);
  }

  apds9960.getColorData(&ColorDistanceData[0], &ColorDistanceData[1], &ColorDistanceData[2], &clear);
  ColorDistanceData[3] =apds9960.readProximity();
}
