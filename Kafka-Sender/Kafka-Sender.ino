/*
 Name:		Kafka_Sender.ino
 Created:	8/19/2019 10:32:55 AM
 Author:	Puppe
*/


//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
//#include <OSCMessage.h>
#include "pw.h"

#define LED_ESP 2
#define JOYSTICK_AXIS_X A0
#define JOYSTICK_AXIS_Y A3

// OSC-Stuff ----------------------------------------------------------
IPAddress outIp;
WiFiUDP Udp;
const unsigned int localPort = 9000;
const unsigned int destPort = 8000;


bool isRunning = false;
uint16_t centerXAxis = 4096 / 2;
uint16_t centerYAxis = 4096 / 2;
uint16_t maxRangeX = centerXAxis - 1;
uint16_t maxRangeY = centerYAxis - 1;
uint16_t tolerance = 10;

//-------------------------------------------------------------

void setup() {

	Serial.begin(115200);

	pinMode(LED_ESP, OUTPUT);       // ESP8266 LED - 2, D4, LED_ESP
	pinMode(LED_BUILTIN, OUTPUT);   // NodeMCU LED - 16, D0, LED_BUILTIN, BUILTIN_LED

	digitalWrite(LED_ESP, HIGH);	// Turn ESP's tiny LED off at startup (anyway, that pin is used for the tb6612fng
	digitalWrite(LED_BUILTIN, LOW);	// turn IR and Control-LED ON at startup!

	centerXAxis = analogRead(JOYSTICK_AXIS_X);
	centerYAxis = analogRead(JOYSTICK_AXIS_Y);

	maxRangeX = centerXAxis > (4096 / 2) ? 4096 - centerXAxis : centerXAxis;
	maxRangeY = centerYAxis > (4096 / 2) ? 4096 - centerYAxis : centerYAxis;

	// Connect to WiFi
	// ---------------
	Serial.println("");
	Serial.print("Connecting to ");
	Serial.println(ssid);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Udp.begin(localPort);
	Serial.println();
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	Serial.print("Listening at port: ");
	//Serial.println(Udp.localPort());
	Serial.println(localPort);

	// OSC-Stuff
	if (!outIp.fromString("192.168.4.1"))
	{
		Serial.println("can't parse Remote-IP-Address.");
	}
	else {
		Serial.print("outIp: ");
		Serial.println(outIp.toString());
	}
}

// the loop function runs over and over again until power down or reset
void loop() {
	int xAxis = analogRead(JOYSTICK_AXIS_X);
	int yAxis = analogRead(JOYSTICK_AXIS_Y);
	

	xAxis = map(xAxis, centerXAxis - maxRangeX, centerXAxis + maxRangeX, -100, 100);		// 12-Bit ADC on ESP32
	yAxis = map(yAxis, centerYAxis - maxRangeY, centerYAxis + maxRangeY, -100, 100);
	xAxis = constrain(xAxis, -100, 100);
	yAxis = constrain(yAxis, -100, 100);

	//Serial.print(xAxis);
	//Serial.print("\t");
	//Serial.println(yAxis);


	if ((abs(yAxis) < tolerance) && (abs(xAxis) < tolerance) && isRunning) {
		stop();
	}
	else if ((abs(yAxis) > tolerance) || (abs(xAxis) > tolerance)) {
		int d1Clamp = constrain(map(xAxis, -100, 0, 50, 0), 50, 0);
		int d2Clamp = constrain(map(xAxis, 100, 0, 50, 0), 50, 0);

		int m1 = yAxis - d1Clamp + d2Clamp;
		int m2 = yAxis - d2Clamp + d1Clamp;

		drive(m1, m2);
	}
	delay(50);
}


void stop()
{
	digitalWrite(LED_ESP, HIGH);
	Serial.println("stop");
	OSCMessage msg("/1/stop");
	Udp.beginPacket(outIp, destPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();

	isRunning = false;
}

void drive(int m1, int m2)
{
	digitalWrite(LED_ESP, LOW);
	Serial.print(m1);
	Serial.print("\t");
	Serial.println(m2);

	OSCMessage msg("/1/drive");
	msg.add(m1);
	msg.add(m2);
	Udp.beginPacket(outIp, destPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();

	isRunning = true;
}