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
const char* ssid = "****";
const char* password = "****";
const char* mqtt_server = "lapsoft.mooo.com";
const char* mqtt_login = "esp";
const char* mqtt_password = "L9{HTRfq7#N!";
const int mqtt_port = 10101;
void setupTopics(){
    // subscribe("esp/test", TopicOut);
}
# 28 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino" 2
/*

  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)

  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)

  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)

  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)

  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)

*/
# 39 "c:\\Users\\IVAN\\Desktop\\nto\\lab\\lab.ino"
/* MQTT */
//Функция для оформления подписки




void TopicOut(String s){
  Serial.println(s);
}


void setup(){
  StartAll();
  doorServo.write(90);
  coolerOn();
}
void loop(){

  std::cout<<getToque()<<'\n';

}
