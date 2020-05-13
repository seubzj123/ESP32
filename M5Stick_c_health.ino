#include <M5StickC.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
#include "aliyun_mqtt.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "Protocentral_MAX30205.h"
#include <EEPROM.h>

//初始化蓝牙
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
char *pin = "1234";
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
//变量声明
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int32_t spo2_new = 90; //SPO2_new value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int32_t heartRate_new = 60; //heart new rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
String Hum = "30.50";
String Temp = "25.00";
String CO2 = "400";
String TVOC = "0";
double human_t = 36.4;
String fancon; //风扇状态
String humcon; //加湿器状态
char fan_con = '0';
char hum_con = '0';
String comdata = "";//声明字符串变量

MAX30205 tempSensor;
//screen&clock
#define TFT_GREY 0x5AEB
#define TFTW            80     // screen width
#define TFTH            160     // screen height
#define TFTW2           40     // half screen width
#define TFTH2           80     // half screen height

#define WIFI_SSID        "vivo X23"//替换自己的WIFI
#define WIFI_PASSWD      "b123456789"//替换自己的WIFI
#define PRODUCT_KEY      "a1lAcM3V1ti" //替换自己的PRODUCT_KEY
#define DEVICE_NAME      "M5stick" //替换自己的DEVICE_NAME
#define DEVICE_SECRET    "VQo4KdxY2Jip6LaCeA8GVgVf4lIqa8Xh"//替换自己的DEVICE_SECRET
#define DEV_VERSION       "S-TH-WIFI-v1.0-20190220"        //固件版本信息
#define ALINK_BODY_FORMAT         "{\"id\":\"123\",\"version\":\"1.0\",\"method\":\"%s\",\"params\":%s}"
#define ALINK_TOPIC_PROP_POST     "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/event/property/post"
#define ALINK_TOPIC_PROP_POSTRSP  "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/event/property/post_reply"
#define ALINK_TOPIC_PROP_SET      "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/service/property/set"
#define ALINK_METHOD_PROP_POST    "thing.event.property.post"
#define ALINK_TOPIC_DEV_INFO      "/ota/device/inform/" PRODUCT_KEY "/" DEVICE_NAME ""
#define ALINK_VERSION_FROMA      "{\"id\": 123,\"params\": {\"version\": \"%s\"}}"
unsigned long lastMs = 0;

WiFiClient   espClient;
PubSubClient mqttClient(espClient);

//clock define

float sx1 = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 40, osy = 40, omx = 40, omy = 40, ohx = 40, ohy = 40; // Saved H, M, S x & y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
uint32_t targetTime = 0;                    // for next 1 second timeout
static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time
boolean initial = 1;
//screen define
int button_num = 0;
int cur_value = 0;
int last_value = 0;
const int button = 37;

//字符串处理函数
void num_deal()
{
  int commaPosition;
  int commaPosition_1;
  do {
    commaPosition = comdata.indexOf(',');
    commaPosition_1 = comdata.indexOf(':');
    if (commaPosition != -1)
    {
      TVOC = comdata.substring(commaPosition_1 + 1, commaPosition);
      Serial.println(TVOC);
      comdata = comdata.substring(commaPosition + 1, comdata.length());

      commaPosition = comdata.indexOf(',');
      commaPosition_1 = comdata.indexOf(':');
      Temp = comdata.substring(commaPosition_1 + 1, commaPosition);
      Serial.println(Temp);
      comdata = comdata.substring(commaPosition + 1, comdata.length());

      commaPosition = comdata.indexOf(',');
      commaPosition_1 = comdata.indexOf(':');
      fancon = comdata.substring(commaPosition_1 + 1, commaPosition);
      fan_con = fancon[0];
      Serial.println(fancon);
      comdata = comdata.substring(commaPosition + 1, comdata.length());

      commaPosition = comdata.indexOf(',');
      commaPosition_1 = comdata.indexOf(':');
      Hum = comdata.substring(commaPosition_1 + 1, commaPosition);
      Serial.println(Hum);
      comdata = comdata.substring(commaPosition + 1, comdata.length());

      commaPosition = comdata.indexOf(',');
      commaPosition_1 = comdata.indexOf(':');
      CO2 = comdata.substring(commaPosition_1 + 1, commaPosition);
      Serial.println(CO2);
      comdata = comdata.substring(commaPosition + 1, comdata.length());

      commaPosition = comdata.indexOf(',');
      commaPosition_1 = comdata.indexOf(':');
      humcon = comdata.substring(commaPosition_1 + 1, comdata.length()-1);
      hum_con = humcon[0];
      Serial.println(humcon);
      comdata = comdata.substring(commaPosition + 1, comdata.length());
    }
    else
    {
      if (comdata.length() > 0)
        Serial.println(comdata);
    }
  }
  while (commaPosition >= 0);
}

void init_wifi(const char *ssid, const char *password)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi does not connect, try again ...");
    delay(500);
  }

  Serial.println("Wifi is connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//mqtt回调函数
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  payload[length] = '\0';
  Serial.println((char *)payload);
  comdata = (char *)payload;
  Serial.println(comdata);
  num_deal();
}
//mqtt版本信息上传
void mqtt_version_post()
{
  char param[512];
  char jsonBuf[1024];

  //sprintf(param, "{\"MotionAlarmState\":%d}", digitalRead(13));
  sprintf(param, "{\"id\": 123,\"params\": {\"version\": \"%s\"}}", DEV_VERSION);
  //sprintf(jsonBuf, ALINK_BODY_FORMAT, ALINK_METHOD_PROP_POST, param);
  Serial.println(param);
  mqttClient.publish(ALINK_TOPIC_DEV_INFO, param);
}

void mqtt_check_connect()
{
  while (!mqttClient.connected())//
  {
    while (connect_aliyun_mqtt(mqttClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET))
    {
      Serial.println("MQTT connect succeed!");
      //client.subscribe(ALINK_TOPIC_PROP_POSTRSP);
      mqttClient.subscribe(ALINK_TOPIC_PROP_SET);

      Serial.println("subscribe done");
      mqtt_version_post();
    }
  }

}

void mqtt_interval_post()
{
  char param[512];
  char jsonBuf[1024];

  //sprintf(param, "{\"MotionAlarmState\":%d}", digitalRead(13));
  sprintf(param, "{\"O2Content\":%d,\"HeartRate\":%d,\"BodyTemp\":%.2f}", spo2_new, heartRate_new, human_t);
  sprintf(jsonBuf, ALINK_BODY_FORMAT, ALINK_METHOD_PROP_POST, param);
  Serial.println(jsonBuf);
  mqttClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf);
}

//电子时钟
void clock_show() {

  if (targetTime < millis()) {
    targetTime += 1000;
    ss++;              // Advance second
    if (ss == 60) {
      ss = 0;
      mm++;            // Advance minute
      if (mm > 59) {
        mm = 0;
        hh++;          // Advance hour
        if (hh > 23) {
          hh = 0;
        }
      }
    }

    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss * 6;                // 0-59 -> 0-354
    mdeg = mm * 6 + sdeg * 0.01666667; // 0-59 -> 0-360 - includes seconds
    hdeg = hh * 30 + mdeg * 0.0833333; // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg - 90) * 0.0174532925);
    hy = sin((hdeg - 90) * 0.0174532925);
    mx = cos((mdeg - 90) * 0.0174532925);
    my = sin((mdeg - 90) * 0.0174532925);
    sx1 = cos((sdeg - 90) * 0.0174532925);
    sy = sin((sdeg - 90) * 0.0174532925);

    if (ss == 0 || initial) {
      initial = 0;
      // Erase hour and minute hand positions every minute
      M5.Lcd.drawLine(ohx, ohy, 40, 40, TFT_BLACK);
      ohx = hx * 15 + 40;
      ohy = hy * 15 + 40;
      M5.Lcd.drawLine(omx, omy, 40, 40, TFT_BLACK);
      omx = mx * 20 + 40;
      omy = my * 20 + 40;
    }

    // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
    M5.Lcd.drawLine(osx, osy, 40, 40, TFT_BLACK);
    osx = sx1 * 25 + 40;
    osy = sy * 25 + 40;
    M5.Lcd.drawLine(osx, osy, 40, 40, TFT_RED);
    M5.Lcd.drawLine(ohx, ohy, 40, 40, TFT_WHITE);
    M5.Lcd.drawLine(omx, omy, 40, 40, TFT_WHITE);
    M5.Lcd.drawLine(osx, osy, 40, 40, TFT_RED);
    M5.Lcd.fillCircle(40, 40, 2, TFT_RED);
  }
}

void fillhalf() {
  for (int j = 5; j < 66; j++) {
    M5.Lcd.fillRect(0, TFTH2 + j, TFTW, 1, TFT_BLACK);
  }
}
//屏幕显示
void screen() {
  //up-,down+,left-,right+
  if (button_num % 6 == 1) {
    //  M5.Lcd.fillScreen(TFT_BLACK);
    fillhalf();
    M5.Lcd.fillRect(0, TFTH2 + 5, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 65, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 30, TFTH2 + 23);
    M5.Lcd.println("Temperature");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 20, TFTH2 + 40 );
    M5.Lcd.println(Temp);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 15, TFTH2 + 40 );
    M5.Lcd.println("`C");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( 24, TFTH2 + 10);
    M5.Lcd.println("MODE 1");
  }
  else if (button_num % 6 == 2) {
    // M5.Lcd.fillScreen(TFT_BLACK);
    fillhalf();
    M5.Lcd.fillRect(0, TFTH2 + 5, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 65, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 15, TFTH2 + 23);
    M5.Lcd.println("CO2:");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 22, TFTH2 + 40 );
    M5.Lcd.println(CO2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 8, TFTH2 + 40 );
    M5.Lcd.println("ppm");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( 24, TFTH2 + 10);
    M5.Lcd.println("MODE 2");
  }
  else if (button_num % 6 == 3) {
    //  M5.Lcd.fillScreen(TFT_BLACK);
    fillhalf();
    M5.Lcd.fillRect(0, TFTH2 + 5, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 65, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 25, TFTH2 + 20);
    M5.Lcd.println("Heart Rate");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 18, TFTH2 + 32 );
    M5.Lcd.println(heartRate_new);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 8, TFTH2 + 32 );
    M5.Lcd.println("/min");
    M5.Lcd.fillRect(0, TFTH2 + 40, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 35, TFTH2 + 44);
    M5.Lcd.println("Blood Oxygen");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 21, TFTH2 + 56 );
    M5.Lcd.println(spo2_new);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 15, TFTH2 + 56 );
    M5.Lcd.println("%");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( 24, TFTH2 + 10);
    M5.Lcd.println("MODE 3");
  }
  else if (button_num % 6 == 4) {
    // M5.Lcd.fillScreen(TFT_BLACK);
    fillhalf();
    M5.Lcd.fillRect(0, TFTH2 + 5, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 65, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 25, TFTH2 + 25);
    M5.Lcd.println("Humidity");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 22, TFTH2 + 40 );
    M5.Lcd.println(Hum);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 15, TFTH2 + 40 );
    M5.Lcd.println("%");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( 24, TFTH2 + 10);
    M5.Lcd.println("MODE 4");
  }
  else if (button_num % 6 == 5) {
    // M5.Lcd.fillScreen(TFT_BLACK);
    fillhalf();
    M5.Lcd.fillRect(0, TFTH2 + 5, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 65, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 28, TFTH2 + 22);
    M5.Lcd.println("Fan");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 8, TFTH2 + 22 );
    if (fan_con == '1') M5.Lcd.println("ON");
    else M5.Lcd.println("OFF");
    M5.Lcd.fillRect(0, TFTH2 + 38, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 37, TFTH2 + 45);
    M5.Lcd.println("Humidifier");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 + 12, TFTH2 + 45);
    if (hum_con == '1') M5.Lcd.println("ON");
    else M5.Lcd.println("OFF");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( 24, TFTH2 + 10);
    M5.Lcd.println("MODE 5");
  }
  else if (button_num % 6 == 0) {
    // M5.Lcd.fillScreen(TFT_BLACK);
    fillhalf();
    M5.Lcd.fillRect(0, TFTH2 + 5, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 65, TFTW, 1, TFT_WHITE);
    M5.Lcd.fillRect(0, TFTH2 + 18, TFTW, 1, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 25, TFTH2 + 25);
    M5.Lcd.println("HEALTH");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( TFTW2 - 30, TFTH2 + 40 );
    M5.Lcd.println("INSTRUCMENT");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor( 16, TFTH2 + 10);
    M5.Lcd.println("MY DEVICE");
  }
}
void setup()
{
  // Initialize the M5StickC object
  M5.begin();
  /* initialize serial for debugging */
  Serial.begin(115200);
  //initialize wifi
  init_wifi(WIFI_SSID, WIFI_PASSWD);
  mqttClient.setCallback(mqtt_callback);
  M5.Lcd.print("wifi_connected");
  SerialBT.setPin(pin);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  M5.Lcd.print("bt connected");
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("Demo Start");
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    for (;;);
  }
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //体温传感器初始化
  tempSensor.begin();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 1638

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    for (; particleSensor.available() == false;) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  //screen&clock
  M5.Lcd.fillScreen(TFT_GREY);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_GREY);  // Adding a background colour erases previous text automatically
  M5.Lcd.fillCircle(40, 40, 40, TFT_GREEN);
  M5.Lcd.fillCircle(40, 40, 36, TFT_BLACK);
  for (int i = 0; i < 360; i += 30) {
    sx1 = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx1 * 38 + 40;
    yy0 = sy * 38 + 40;
    x1 = sx1 * 32 + 40;
    yy1 = sy * 32 + 40;
    M5.Lcd.drawLine(x0, yy0, x1, yy1, TFT_GREEN);
  }
  for (int i = 0; i < 360; i += 6) {
    sx1 = cos((i - 90) * 0.0174532925);
    sy = sin((i - 90) * 0.0174532925);
    x0 = sx1 * 34 + 40;
    yy0 = sy * 34 + 40;
    M5.Lcd.drawPixel(x0, yy0, TFT_WHITE);
    if (i == 0 || i == 180) M5.Lcd.fillCircle(x0, yy0, 2, TFT_WHITE);
    if (i == 90 || i == 270) M5.Lcd.fillCircle(x0, yy0, 2, TFT_WHITE);
  }
  M5.Lcd.fillCircle(40, 40, 2, TFT_WHITE);
  targetTime = millis() + 1000;
  //home键设置为中断
  pinMode(M5_BUTTON_HOME, INPUT | PULLUP);
  attachInterrupt(digitalPinToInterrupt(M5_BUTTON_HOME), button_isr, FALLING);
  clock_show();
  delay(50);
  screen();
}

// the loop function runs over and over again forever
void loop()
{
  clock_show();
  delay(100);
  screen();
  if (millis() - lastMs >= 5000)
  {
    lastMs = millis();
    mqtt_check_connect();
    // Post
    mqtt_interval_post();
  }
  mqttClient.loop();
  //MAX30205
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }
  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    for (; particleSensor.available() == false;) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  if (spo2 > 50&&validSPO2==1)spo2_new = spo2;
  if (heartRate > 40&&validHeartRate==1)heartRate_new = heartRate * 0.75;
  //体温数据获取
  human_t = double(tempSensor.getTemperature());
}

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

unsigned long last_isr_time;
#define ISR_DITHERING_TIME_MS 10
// 中断函数
void button_isr() {
  if (millis() - last_isr_time < ISR_DITHERING_TIME_MS) {
    return;
  }
  last_isr_time = millis();
  button_num++;
  screen();
}
