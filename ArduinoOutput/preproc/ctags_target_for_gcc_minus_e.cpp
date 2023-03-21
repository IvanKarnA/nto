# 1 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
# 2 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 3 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
# 4 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
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
Adafruit_APDS9960 apds9960;
#define ColorDistanceSensorAddr 0x07
uint16_t ColorDistanceData[4];

/*

  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)

  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)

  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)

  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)

  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)

*/
# 25 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
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

void setup(){
  ColorDistanceSensorBegin();


}
void loop(){
  ColorDistanceGetData();
  std::cout<<ColorDistanceData[0]<<" "<<ColorDistanceData[2]<<" "<<ColorDistanceData[2]<<" "<<ColorDistanceData[3]<<"\n";
  delay(100);
}
