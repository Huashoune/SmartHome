#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Ticker.h>
#include <BH1750.h>

const char *wifissid = "esp";                    //WIfiID
const char *password = "yyh729826";              //WifiPassword
const char *mqttServer = "43.143.48.34";         //MqttIp
const char *PubTopic_TEMP = "ESP32/DHT22/TEMP";  //ESP32发送温度
const char *PubTopic_HUMI = "ESP32/DHT22/HUMI";  //ESP32发送湿度
const char *PubTopic_BH1750 = "ESP32/BH1750";    //ESP32发送光照度
const char *PubTopic_MQ4 = "ESP32/MQ4";          //ESP32发送天然气传感器状态
const char *PubTopic_HCSR501 = "ESP32/HCSR501";  //ESP32发送人体红外传感器状态
const char *clientId = "ESP32_Sensor";
const char *user = "esp32";
const char *pass = "yyh729826";
const char *willTopic = "willTopic";  // 遗嘱主题名称
const int willQos = 1;                // 遗嘱QoS
const char *willMessage = "willMsg";  // 遗嘱主题信息
boolean willRetain = false;           // 遗嘱保留
const int subQoS = 1;                 // 客户端订阅主题时使用的QoS级别（截止2020-10-07，仅支持QoS = 1，不支持QoS = 2）
boolean cleanSession = false;         // 清除会话（如QoS>0必须要设为false）

const int Led = 2;
//DHT22定义
#define DHTPIN 13
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
//BH1750定义
#define BH1750SCL 22
#define BH1750SDA 21
//MQ4定义
const int AOUTpin = 12;  //MQ-4 传感器的 AOUT 引脚进入 arduino 的模拟引脚 A1。
const int DOUTpin = 14;  //MQ-4 传感器的 DOUT 引脚进入 arduino 的数字引脚 D7。
int MQ4limit;
int MQ4value;
//HC-SR501定义
int inPin = 4;  //HC-SR501人体紅外传感器检测OUT信引脚 连接到ESP32 GPIO17
int HCSR501val;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
BH1750 bh1750;

Ticker TimerDHT22;    //定时器,用来循环上传DHT数据
Ticker TimerBH1750;   //定时器,用来循环上传DHT数据
Ticker TimerMQ4;      //定时器,用来循环上传MQ4数据
Ticker TimerHCSR501;  //定时器,用来循环上传HCSR501数据

void LedOpen() {
  Serial.println("指示灯亮");
  digitalWrite(Led, HIGH);
}

void LedClose() {
  Serial.println("指示灯灭");
  digitalWrite(Led, LOW);
}

void setup_wifi() {
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("正在连接至Wifi:");
    Serial.println(wifissid);
    delay(2500);
    WiFi.begin(wifissid, password);
  }
  Serial.print("已连接 ");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// 连接MQTT服务器并订阅信息
void connectMQTTserver() {
  /* 连接MQTT服务器
    boolean connect(const char* id, const char* user,
                  const char* pass, const char* willTopic,
                  uint8_t willQos, boolean willRetain,
                  const char* willMessage, boolean cleanSession);
    若让设备在离线时仍然能够让qos1工作，则connect时的cleanSession需要设置为false
  */
  if (mqttClient.connect(clientId, user, pass, willTopic, willQos, willRetain, willMessage, cleanSession)) {
    Serial.print("MQTT Server Connected. ClientId: ");
    Serial.println(clientId);
    // subscribeTopic();  // 订阅指定主题
  } else {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqttClient.state());
    delay(5000);
  }
}

void receiveCallback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    // Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
}

void connectCheck() {
  if (WiFi.status() == WL_CONNECTED || mqttClient.connected()) {
    LedOpen();
  } else {
    LedClose();
  }
}
//上传DHT22数据
void sendTempAndHumi() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // 检查是否有任何读取失败并提前退出（重试）。
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  mqttClient.publish(PubTopic_HUMI, String(h).c_str(), false);
  mqttClient.publish(PubTopic_TEMP, String(t).c_str(), false);

  Serial.print(F("湿度: "));
  Serial.print(h);
  Serial.print(F("%  温度: "));
  Serial.print(t);
  Serial.println(F("°C "));
  delay(500);
}
//上传BH750数据
void sendLux() {
  float lux = bh1750.readLightLevel();
  mqttClient.publish(PubTopic_BH1750, String(lux).c_str(), false);
  Serial.print("光照值: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(500);
}
//上传MQ4数据
void sendMq4() {
  MQ4value = analogRead(AOUTpin);   //从 MQ-4 传感器的 AOUT 引脚读取模拟值。
  MQ4limit = digitalRead(DOUTpin);  //从 MQ-4 传感器的 DOUT 引脚读取数字值。
  if (MQ4limit == HIGH) {
    Serial.print("天然气：");
    Serial.println("安全");
    mqttClient.publish(PubTopic_MQ4, "安全", false);
    delay(500);
  } else {
    Serial.print("天然气：");
    Serial.println("危险");
    mqttClient.publish(PubTopic_MQ4, "危险", false);
    delay(500);
  }
}
//上传HCSR501数据
void Hcsr501() {
  int HCSR501val = digitalRead(inPin);  //人體紅外線感測器讀出數位值
  if (HCSR501val == HIGH)           // 如果有人在移動
  {
    Serial.print("人体红外：");
    Serial.println("有人");
    mqttClient.publish(PubTopic_HCSR501, "有人", false);
    delay(500);
  } else {
    Serial.print("人体红外：");
    Serial.println("无人");
    mqttClient.publish(PubTopic_HCSR501, "无人", false);
    delay(500);
  }
}
void setup() {
  Serial.begin(115200);
  setup_wifi();
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(receiveCallback);
  connectMQTTserver();
  connectCheck();

  pinMode(Led, OUTPUT);   //将引脚设置为 arduino 的输出。
  pinMode(inPin, INPUT);  //设置inPin对应的脚GPIO17为输入

  dht.begin();
  Wire.begin();
  bh1750.begin();

  TimerDHT22.attach(1, sendTempAndHumi);  //定时每5秒调用一次发送数据函数sendTempAndHumi
  TimerBH1750.attach(1, sendLux);         //定时每5秒调用一次发送数据函数sendLux
  TimerHCSR501.attach(1, Hcsr501);        //定时每1秒调用一次发送数据函数TimerHCSR501
  TimerMQ4.attach(1, sendMq4);            //定时每1秒调用一次发送数据函数sendMq4
}

void loop() {
  if (mqttClient.connected()) {  // 如果开发板成功连接服务器
    mqttClient.loop();           // 处理信息以及心跳
  } else {                       // 如果开发板未能成功连接服务器
    connectMQTTserver();         // 则尝试连接服务器
  }
}