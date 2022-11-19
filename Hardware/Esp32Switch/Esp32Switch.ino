#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Stepper.h>
#include <Ticker.h>
#include "OneButton.h"

const char *wifissid = "esp";            // Wifi名称
const char *password = "yyh729826";      // Wifi密码
const char *mqttServer = "43.143.48.34"; // MQTT服务器
const char *subTopic_Switch = "ESP32/Switch";
const char *PubTopic_FanUi = "ESP32/Ui/Fan";               // ESP32发送Fan状态
const char *PubTopic_CurtainUi = "ESP32/Ui/Curtain";       // ESP32发送Curtain状态
const char *PubTopic_HumidifierUi = "ESP32/Ui/Humidifier"; // ESP32发送Humidifier状态
const char *PubTopic_LampUi = "ESP32/Ui/Lamp";             // ESP32发送Lampn状态
const char *PubTopic_AccessUi = "ESP32/Ui/Access";         // ESP32发送Access状态
const char *PubTopic_BuzzerUi = "ESP32/Ui/Buzzer";         // ESP32发送Buzzer状态
const char *PubTopic_AllUi = "ESP32/Ui/All";               // ESP32发送Buzzer状态
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char *clientId = "ESP32_Switch";
const char *user = "esp32";
const char *pass = "yyh729826";
const char *willTopic = "willTopic"; // 遗嘱主题名称
const int willQos = 1;               // 遗嘱QoS
const char *willMessage = "willMsg"; // 遗嘱主题信息
boolean willRetain = false;          // 遗嘱保留
const int subQoS = 1;                // 客户端订阅主题时使用的QoS级别（截止2020-10-07，仅支持QoS = 1，不支持QoS = 2）
boolean cleanSession = false;        // 清除会话（如QoS>0必须要设为false）

const int Led = 2;
const int Fan = 13;
// const int Curtain = 12; //窗帘
const int Humidifier = 12;
const int Lamp = 14;
const int Buzzer = 4;

const int Spare = 18; //继电器4口备用

// Sg90舵机
int freq = 50;           // 频率(20ms周期)
int channel = 8;         // 通道(高速通道（0 ~ 7）由80MHz时钟驱动，低速通道（8 ~ 15）由 1MHz 时钟驱动。)
int resolution = 8;      // 分辨率
const int Access = 15;   //门禁
const int Access_Ui = 5; //借助GPIO5更新门禁状态

// ULN2003
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
const int stepsPerRevolution = 1024; // 定义步进电机转的步数
//这里特别注意 ，后面4个参数分别是驱动板上的 IN1 , IN3 , IN2 , IN4
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);
const int Curtain_Ui = 19; //借助GPIO18更新窗帘状态

#define BUTTON_PIN 21 //将按钮引脚绑定到GPIO 12

OneButton button(BUTTON_PIN, true);

boolean ULN2003State = false; // ULN2003状态 false关 true开
void CurtaiState()
{
  int CurtairState = digitalRead(Curtain_Ui);
  if (CurtairState == HIGH)
  {
    ULN2003State = true;
  }
  else
  {
    ULN2003State = false;
  }
}

void FanUi()
{
  int Fanval = digitalRead(Fan);
  if (Fanval == HIGH)
  {
    // Serial.println("风扇状态：开");
    mqttClient.publish(PubTopic_FanUi, "FanOpen", false);
  }
  else
  {
    // Serial.println("风扇状态：关");
    mqttClient.publish(PubTopic_FanUi, "FanClose", false);
  }
}
void CurtaiUi()
{
  int Curtairval = digitalRead(Curtain_Ui);
  if (Curtairval == HIGH)
  {
    // Serial.println("窗帘状态：开");
    mqttClient.publish(PubTopic_CurtainUi, "CurtainOpen", false);
  }
  else
  {
    // Serial.println("窗帘状态：关");
    mqttClient.publish(PubTopic_CurtainUi, "CurtainClose", false);
  }
}
void HumidifierUi()
{
  int Humidifierval = digitalRead(Humidifier);
  if (Humidifierval == HIGH)
  {
    // Serial.println("加湿器状态：开");
    mqttClient.publish(PubTopic_HumidifierUi, "HumidifierOpen", false);
  }
  else
  {
    // Serial.println("加湿器状态：关");
    mqttClient.publish(PubTopic_HumidifierUi, "HumidifierClose", false);
  }
}
void LampUi()
{
  int Lampval = digitalRead(Lamp);
  if (Lampval == HIGH)
  {
    // Serial.println("射灯状态：开");
    mqttClient.publish(PubTopic_LampUi, "LampOpen", false);
  }
  else
  {
    // Serial.println("射灯状态：关");
    mqttClient.publish(PubTopic_LampUi, "LampClose", false);
  }
}
void AccessUi()
{
  int Accessval = digitalRead(Access_Ui);
  if (Accessval == HIGH)
  {
    // Serial.println("门禁状态：开");
    mqttClient.publish(PubTopic_AccessUi, "AccessOpen", false);
  }
  else
  {
    // Serial.println("门禁状态：关");
    mqttClient.publish(PubTopic_AccessUi, "AccessClose", false);
  }
}
void BuzzerUi()
{
  int Buzzerval = digitalRead(Buzzer);
  if (Buzzerval == HIGH)
  {
    // Serial.println("蜂鸣器状态：开");
    mqttClient.publish(PubTopic_BuzzerUi, "BuzzerOpen", false);
  }
  else
  {
    // Serial.println("蜂鸣器状态：关");
    mqttClient.publish(PubTopic_BuzzerUi, "BuzzerClose", false);
  }
}
void LedOpen()
{
  Serial.println("指示灯亮");
  digitalWrite(Led, HIGH);
}
void FanOpen()
{
  Serial.println("开风扇");
  digitalWrite(Fan, HIGH);
}
void CurtainOpen()
{
  Serial.println("开窗帘");
  myStepper.step(stepsPerRevolution);
}
void HumidifierOpen()
{
  Serial.println("开加湿器");
  digitalWrite(Humidifier, HIGH);
}
void LampOpen()
{
  Serial.println("开射灯");
  digitalWrite(Lamp, HIGH);
}
void AccessOpen()
{
  Serial.println("开门");
  ledcWrite(channel, calculatePWM(180)); // 输出PWM
}
void BuzzerOpen()
{
  Serial.println("蜂鸣器开");
  digitalWrite(Buzzer, HIGH);
}
void LedClose()
{
  Serial.println("指示灯灭");
  digitalWrite(Led, LOW);
}
void FanClose()
{
  Serial.println("关风扇");
  digitalWrite(Fan, LOW);
}
void CurtainClose()
{
  Serial.println("关窗帘");
  myStepper.step(-stepsPerRevolution);
}
void HumidifierClose()
{
  Serial.println("关加湿器");
  digitalWrite(Humidifier, LOW);
}
void LampClose()
{
  Serial.println("关射灯");
  digitalWrite(Lamp, LOW);
}
void AccessClose()
{
  Serial.println("关门");
  ledcWrite(channel, calculatePWM(0)); // 输出PWM
}
void BuzzerClose()
{
  Serial.println("蜂鸣器关");
  digitalWrite(Buzzer, LOW);
}
int calculatePWM(int degree)
{ // 0-180度
  // 20ms周期，高电平0.5-2.5ms，对应0-180度角度
  const float deadZone = 6.4; //对应0.5ms（0.5ms/(20ms/256）)
  const float max = 32;       //对应2.5ms
  if (degree < 0)
    degree = 0;
  if (degree > 180)
    degree = 180;
  return (int)(((max - deadZone) / 180) * degree + deadZone);
}
void BuzzerSwitch()
{
  int Buzzerval = digitalRead(Buzzer);
  if (Buzzerval == HIGH)
  {
    BuzzerClose();
    mqttClient.publish(PubTopic_BuzzerUi, "BuzzerClose", false);
  }
  else
  {
    BuzzerOpen();
    mqttClient.publish(PubTopic_BuzzerUi, "BuzzerOpen", false);
  }
}
//风扇
void BuzzerClick()
{
  Serial.println("蜂鸣器机械开关");
  BuzzerSwitch();
}
//按钮检测状态子程序
void button_attach_loop()
{
  //不断检测按钮按下状态
  button.tick();
}
//回调函数绑定子程序
void button_event_init()
{
  button.reset();                  //清除一下按钮状态机的状态
  button.setDebounceTicks(20);     //设置消抖时长为80毫秒,默认值为：50毫秒
  button.setClickTicks(100);       //设置单击时长为500毫秒,默认值为：400毫秒
  button.attachClick(BuzzerClick); //初始化单击回调函数
}
void setup_wifi()
{
  while (WiFi.status() != WL_CONNECTED)
  {
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
void connectMQTTserver()
{
  /* 连接MQTT服务器
    boolean connect(const char* id, const char* user,
                  const char* pass, const char* willTopic,
                  uint8_t willQos, boolean willRetain,
                  const char* willMessage, boolean cleanSession);
    若让设备在离线时仍然能够让qos1工作，则connect时的cleanSession需要设置为false
  */
  if (mqttClient.connect(clientId, user, pass, willTopic, willQos, willRetain, willMessage, cleanSession))
  {
    Serial.print("MQTT Server Connected. ClientId: ");
    Serial.println(clientId);
    subscribeTopic(); // 订阅指定主题
  }
  else
  {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqttClient.state());
    delay(5000);
  }
}
void receiveCallback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    // Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
  if (String(topic) == subTopic_Switch)
  {
    if (messageTemp == "Ui_State")
    {
      FanUi();
      CurtaiUi();
      HumidifierUi();
      LampUi();
      AccessUi();
      BuzzerUi();
    }
    if (messageTemp == "Fan_Open")
    {
      mqttClient.publish(PubTopic_FanUi, "FanOpen", false);
      FanOpen();
    }
    if (messageTemp == "Curtain_Open")
    {
      mqttClient.publish(PubTopic_CurtainUi, "CurtainOpen", false);
      CurtaiState();
      if (!ULN2003State)
      {
        CurtainOpen();
      }
      digitalWrite(Curtain_Ui, HIGH);
    }
    if (messageTemp == "Humidifier_Open")
    {
      mqttClient.publish(PubTopic_HumidifierUi, "HumidifierOpen", false);
      HumidifierOpen();
    }
    if (messageTemp == "Lamp_Open")
    {
      mqttClient.publish(PubTopic_LampUi, "LampOpen", false);
      LampOpen();
    }
    if (messageTemp == "Access_Open")
    {
      mqttClient.publish(PubTopic_AccessUi, "AccessOpen", false);
      AccessOpen();
      digitalWrite(Access_Ui, HIGH);
    }
    if (messageTemp == "Buzzer_Open")
    {
      mqttClient.publish(PubTopic_BuzzerUi, "BuzzerOpen", false);
      BuzzerOpen();
    }
    if (messageTemp == "All_Open")
    {
      mqttClient.publish(PubTopic_FanUi, "FanOpen", false);
      FanOpen();
      mqttClient.publish(PubTopic_HumidifierUi, "HumidifierOpen", false);
      HumidifierOpen();
      mqttClient.publish(PubTopic_LampUi, "LampOpen", false);
      LampOpen();
      mqttClient.publish(PubTopic_AccessUi, "AccessOpen", false);
      AccessOpen();
      mqttClient.publish(PubTopic_BuzzerUi, "BuzzerOpen", false);
      BuzzerOpen();
      mqttClient.publish(PubTopic_CurtainUi, "CurtainOpen", false);
      CurtainOpen();
    }
    if (messageTemp == "Fan_Close")
    {
      mqttClient.publish(PubTopic_FanUi, "FanClose", false);
      FanClose();
    }
    if (messageTemp == "Curtain_Close")
    {
      mqttClient.publish(PubTopic_CurtainUi, "CurtainClose", false);
      CurtaiState();
      if (ULN2003State)
      {
        CurtainClose();
      }
      digitalWrite(Curtain_Ui, LOW);
    }
  }
  if (messageTemp == "Humidifier_Close")
  {
    mqttClient.publish(PubTopic_HumidifierUi, "HumidifierClose", false);
    HumidifierClose();
  }
  if (messageTemp == "Lamp_Close")
  {
    mqttClient.publish(PubTopic_LampUi, "LampClose", false);
    LampClose();
  }
  if (messageTemp == "Access_Close")
  {
    digitalWrite(Access_Ui, LOW);
    mqttClient.publish(PubTopic_AccessUi, "AccessClose", false);
    AccessClose();
  }
  if (messageTemp == "Buzzer_Close")
  {
    mqttClient.publish(PubTopic_BuzzerUi, "BuzzerClose", false);
    BuzzerClose();
  }
  if (messageTemp == "All_Close")
  {
    mqttClient.publish(PubTopic_FanUi, "FanClose", false);
    FanClose();
    mqttClient.publish(PubTopic_HumidifierUi, "HumidifierClose", false);
    HumidifierClose();
    mqttClient.publish(PubTopic_LampUi, "LampClose", false);
    LampClose();
    mqttClient.publish(PubTopic_AccessUi, "AccessClose", false);
    AccessClose();
    mqttClient.publish(PubTopic_BuzzerUi, "BuzzerClose", false);
    BuzzerClose();
    mqttClient.publish(PubTopic_CurtainUi, "CurtainClose", false);
    CurtaiState();
    if (ULN2003State)
    {
      CurtainClose();
    }
    digitalWrite(Curtain_Ui, LOW);
  }
}
// 订阅指定主题
void subscribeTopic()
{
  // 建立订阅主题。
  mqttClient.subscribe(subTopic_Switch, subQoS);
  // 通过串口监视器输出是否成功订阅主题以及订阅的主题名称
  if (mqttClient.subscribe(subTopic_Switch, subQoS))
  {
    Serial.print("Subscribe Topic:");
    Serial.println(subTopic_Switch);
  }
  else
  {
    Serial.println("Subscribe Fail...");
  }
}
void connectCheck()
{
  if (WiFi.status() == WL_CONNECTED || mqttClient.connected())
  {
    LedOpen();
  }
  else
  {
    LedClose();
  }
}
void setup()
{
  Serial.begin(115200);
  setup_wifi();
  // 连接MQTT服务器
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(receiveCallback);
  connectMQTTserver();
  connectCheck();

  //设置设备引脚状态
  pinMode(Led, OUTPUT);
  pinMode(Fan, OUTPUT);
  pinMode(Humidifier, OUTPUT);
  pinMode(Lamp, OUTPUT);
  pinMode(Access, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(Spare, OUTPUT);

  myStepper.setSpeed(8); // 设置步进速度使用设定速度方法。步进速度以 rpm 为单位

  ledcSetup(channel, freq, resolution); // 设置通道
  ledcAttachPin(Access, channel);       // 将通道与对应的引脚连接

  pinMode(Access_Ui, OUTPUT);  //门禁状态更新
  pinMode(Curtain_Ui, OUTPUT); //窗帘状态更新

  button_event_init(); //按钮事件初始化
}
void loop()
{
  if (mqttClient.connected())
  {                    // 如果开发板成功连接服务器
    mqttClient.loop(); // 处理信息以及心跳
  }
  else
  {                      // 如果开发板未能成功连接服务器
    connectMQTTserver(); // 则尝试连接服务器
  }
  //不断检测按钮按下状态
  button_attach_loop();
}
