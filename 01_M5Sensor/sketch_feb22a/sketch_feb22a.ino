// M5StickCPlus library
// https://github.com/m5stack/M5StickC-Plus
#include <M5StickCPlus.h>
#include "Ultrasonic.h"

float distance = 0.0F;
float sensor_x = 0.0F;

//  OSC library
//  https://github.com/CNMAT/OSC

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>

// OSC
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>#include "Ultrasonic.h"
 
Ultrasonic ultrasonic(33);

WiFiUDP Udp;
const unsigned int _local_port = 8888;  // local port
const unsigned int _out_port = 9999;    //destination

OSCErrorCode error;

// WiFi
int address = 0;
String ssid = "ccbt_gakki_wifi";
String password = "deaihatotuzen";
String local_ip_str = "10.0.0.100";
String gateway_ip_str = "10.0.0.1";
String out_ip_str = "10.0.0.200";

// String ssid = "pixel_8151";
// String password = "21ceb87671b9";
// String local_ip_str = "192.168.6.100";
// String gateway_ip_str = "192.168.6.1";
// String out_ip_str = "192.168.6.42";

String subnet_ip_str = "255.255.255.0";

IPAddress _local_ip;
IPAddress _gateway;
IPAddress _subnet;
IPAddress _out_ip;

// FreeRTOS
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

void TaskOsc(void *pvParameters);
void TaskDisplay(void *pvParameters);
void TaskSerial(void *pvParameters);

void setup() {
  M5.begin();             // Init M5StickC Plus
  M5.Imu.Init();          // Init IMU
  M5.Lcd.setRotation(3);  // Rotate the screen.
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);

  Serial.begin(115200);

  _local_ip.fromString(local_ip_str);
  _gateway.fromString(gateway_ip_str);
  _subnet.fromString(subnet_ip_str);
  _out_ip.fromString(out_ip_str);

  WiFi.mode(WIFI_STA);
  if (!WiFi.config(_local_ip, _gateway, _subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(_local_port);
  Serial.print("Local port: ");
  Serial.println(_local_port);

  xTaskCreateUniversal(
    TaskSerial, "TaskSerial", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreateUniversal(
     TaskOsc, "TaskOsc", 8192, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
  xTaskCreateUniversal(
    TaskDisplay, "TaskDisplay", 8192, NULL, 4, NULL, ARDUINO_RUNNING_CORE);
}

// mapがうまく機能しなかったので、関数を用意しました
float mapRange (float value, float a, float b, float c, float d) {
    value = (value - a) / (b - a);
    return c + value * (d - c);
}

// OSCタスク
void TaskOsc(void *pvParameters) {
  (void)pvParameters;
  while (1) {

    OSCMessage msg("/sensor");

    // センサの範囲を、0〜1に整える
    //sensor_x = mapRange(distance, 0, 60, 0, 1);
    // 範囲を振り切ってしまわないように
    if(sensor_x<0)sensor_x = 0;
    if(sensor_x>1)sensor_x = 1;
    
    //msg.add(sin(sensor_x));
    msg.add(distance);

    Udp.beginPacket(_out_ip, _out_port);
    msg.send(Udp);    // send the bytes to the SLIP stream
    Udp.endPacket();  // mark the end of the OSC Packet
    msg.empty();      // free space occupied by message
    vTaskDelay(100);
  }
}

// Displayタスク
void TaskDisplay(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    // 加速度センサ
    // M5.IMU.getAccelData(&pitch, &roll, &yaw);
    //distance = ultrasonic.MeasureInInches()
    //distance = ultrasonic.MeasureInCentimeters();
    distance = ultrasonic.MeasureInMillimeters();
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.printf(" %5.2f   ", distance);
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf(" %5.2f   ", sensor_x);

    vTaskDelay(10);
  }
}

// Serial 
void TaskSerial(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    // Send Serial for Recording, to TouchDesigner device
    Serial.printf("%5.2f", distance);
    vTaskDelay(100);
  }
}


void loop() {
  //vTaskDelay(1);
}