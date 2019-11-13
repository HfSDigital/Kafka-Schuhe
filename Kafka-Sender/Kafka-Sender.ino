/*
  Name:		Kafka_Sender.ino
  Created:	8/19/2019 10:32:55 AM
  Author:	Puppe
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include "pw.h"
#include "averageFilter.h"

#define LED_WIFI_OK 2
#define LED_MSG_SENT A17

#define JOYSTICK_AXIS_X A0	// xAxis: throttle
#define JOYSTICK_AXIS_Y A3	// yAxis: steering

#define POTI_STEERING_ADJUST A4

const char* deviceName = "REMOTE_A";

averageFilter avX, avY;

// OSC-Stuff ----------------------------------------------------------
String receiverIP = "192.168.234.223";
IPAddress outIp;
WiFiUDP Udp;
const unsigned int localPort = 9000;
const unsigned int destPort = 8000;


float tolerance = 0.1;
float maxSpeed = 100;
bool isRunning = false;
uint16_t centerXAxis = 0;
uint16_t centerYAxis = 0;
bool isCalibrated = false;
bool reverseGear = false;
unsigned long lastTimeMessageSent, messageSendInterval = 40;
bool ledMsgSentStatus = HIGH;
unsigned long lastTimeLedMsgSentStatusChanged, ledMsgSentStatusInterval = 30;

//-------------------------------------------------------------

void setup() {
  delay(50);
  Serial.begin(115200);

  pinMode(LED_WIFI_OK, OUTPUT);       // ESP8266 LED - 2, D4, LED_ESP
  pinMode(LED_MSG_SENT, OUTPUT);
  digitalWrite(LED_WIFI_OK, ledMsgSentStatus);	// Turn ESP's tiny LED off at startup (anyway, that pin is used for the tb6612fng
  digitalWrite(LED_MSG_SENT, ledMsgSentStatus);	// Turn ESP's tiny LED off at startup (anyway, that pin is used for the tb6612fng

  // Connect to WiFi
  // ---------------
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //WiFi.disconnect();				//Prevent connecting to wifi based on previous configuration
  WiFi.setHostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
  //WiFi.config(staticIP, subnet, gateway, dns);
  WiFi.mode(WIFI_STA); //WiFi mode station (connect to wifi router only)
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_WIFI_OK, HIGH);
    delay(100);
    digitalWrite(LED_WIFI_OK, LOW);
    delay(100);
    Serial.print(".");
  }
  Serial.print("local IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.print("Listening at port: ");
  Serial.println(localPort);

  // OSC
  if (!outIp.fromString(receiverIP))
  {
    Serial.println("can't parse Remote-IP-Address.");
  }
  else {
    Serial.print("receiverIP: ");
    Serial.println(outIp.toString());
  }

  lastTimeMessageSent = millis();
  lastTimeLedMsgSentStatusChanged = millis();
}

//-------------------------------------------------------------

void loop() {
  // read potis
  float xA = analogRead(JOYSTICK_AXIS_X); // xAxis: throttle
  float yA = analogRead(JOYSTICK_AXIS_Y); // yAxis: steering

  // turn off the status LED for the "signal sent" status
  if (millis() - lastTimeLedMsgSentStatusChanged > ledMsgSentStatusInterval)
  {
    ledMsgSentStatus = HIGH;
  }
  digitalWrite(LED_MSG_SENT, ledMsgSentStatus);

  // check WiFi-Connection
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_WIFI_OK, HIGH);
    delay(100);
    digitalWrite(LED_WIFI_OK, LOW);
    delay(100);
    Serial.print("WiFi.status != WL_CONNECTED - ");
    Serial.println(WiFi.status());
  }
  else {
    digitalWrite(LED_WIFI_OK, LOW);
  }


  if (!isCalibrated)
  {
    digitalWrite(LED_MSG_SENT, LOW);
    delay(3000);
    centerXAxis = xA;
    centerYAxis = yA;
    Serial.print("cx: ");
    Serial.print(centerXAxis);
    Serial.print("\tcy: ");
    Serial.println(centerYAxis);
    digitalWrite(LED_MSG_SENT, HIGH);
    isCalibrated = true;
  }
  else {
    // map potis from center to 1 and -1
    float xAxis = (xA > centerXAxis) ? mapf(xA, centerXAxis, 4095, 0, 1.0) : mapf(xA, centerXAxis, 0, 0, -1.0);
    float yAxis = (yA > centerYAxis) ? mapf(yA, centerYAxis, 4095, 0, 1.0) : mapf(yA, centerYAxis, 0, 0, -1.0);

    xAxis = avX.filter(xAxis);
    yAxis = avY.filter(yAxis);

    xAxis = constrain(mapf(xAxis, -0.5, 0.5, -1.0, 1.0), -1.0, 1.0);
    yAxis = constrain(mapf(yAxis, -0.5, 0.5, -1.0, 1.0), -1.0, 1.0);

    Serial.print(xAxis);
    Serial.print(",");
    Serial.print(yAxis);
    Serial.print(",");
    Serial.print(tolerance);
    Serial.print(",");
    Serial.println(-tolerance);

    xAxis *= -1; // reverse throttle
    yAxis *= -1;
    
    if ((absf(yAxis) < tolerance) && (absf(xAxis) < tolerance) && isRunning) {
      stop();
    }
    else if ((absf(yAxis) > tolerance) || (absf(xAxis) > tolerance)) {

      float m1 = xAxis + yAxis;
      float m2 = xAxis - yAxis;

      m1 = constrain(m1, -1.0, 1.0);
      m2 = constrain(m2, -1.0, 1.0);

      m1 = mapf(m1, -1.0, 1.0, -maxSpeed, maxSpeed);
      m2 = mapf(m2, -1.0, 1.0, -maxSpeed, maxSpeed);
      if (millis() - lastTimeMessageSent > messageSendInterval) {
        drive(int(m2), int(m1));
      }
    }
  }


}


void stop()
{
  Serial.println("stop");
  OSCMessage msg("/1/stop");
  Udp.beginPacket(outIp, destPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  ledMsgSentStatus = LOW;
  lastTimeMessageSent = millis();
  lastTimeLedMsgSentStatusChanged = millis();
  isRunning = false;
}


void drive(int m1, int m2)
{
  int steeringAdjust = analogRead(POTI_STEERING_ADJUST);
  steeringAdjust = map(steeringAdjust, 0, 4095, -25, 25);
  steeringAdjust = 0;
  if (reverseGear) steeringAdjust *= -1;

  m1 -= steeringAdjust;
  m2 += steeringAdjust;

  m1 = constrain(m1, -100, 100);
  m2 = constrain(m2, -100, 100);
  /*
    Serial.print(m1);
    Serial.print(",");
    Serial.println(m2);
  */
  OSCMessage msg("/1/drive");
  msg.add(m1);
  msg.add(m2);
  Udp.beginPacket(outIp, destPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  ledMsgSentStatus = LOW;
  lastTimeMessageSent = millis();
  lastTimeLedMsgSentStatusChanged = millis();

  isRunning = true;
}


// the map-function seems to be broken on floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// the abs-function seems to be broken on floats
float absf(float val)
{
  return val > 0 ? val : val * -1;
}


float powSigned(float base, float exp)
{
  bool isSigned = base < 0 ? true : false;

  if (isSigned)
    base *= -1.0;

  float p = pow(base, exp);

  if (isSigned)
    return -p;
  else
    return p;
}
