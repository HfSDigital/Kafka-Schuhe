/*
 Name:		Kafka_Receiver.ino
 Created:	8/19/2019 10:24:52 AM
 Author:	Puppe
*/



//#define USE_SOFTAP
//#define OLD_VERSION

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include "pw.h"
#include "TB6612FNG.h"


#define LED_WIFI_OK D4
#define LED_BATTERY_LOW 10
#define LED_BATTERY_OK 9
#define LED_MESSAGE_RECEIVED D0

#ifndef USE_SOFTAP
	IPAddress staticIP(192, 168, 234, 100); //ESP static ip
	IPAddress gateway(192, 168, 234, 1);   //IP Address of your WiFi Router (Gateway)
	IPAddress subnet(255, 255, 255, 0);  //Subnet mask
	IPAddress dns(8, 8, 8, 8);  //DNS
	const char* deviceName = "SHOE_A";
#endif // !USE_SOFTAP



// OSC-Stuff ----------------------------------------------------------
IPAddress outIp;
WiFiUDP Udp;
const unsigned int localPort = 8000;
const unsigned int destPort = 9000;



// Motor -------------------------------------------------------------
tb6612fng motors;

// Status Monitoring -------------------------------------------------------------
bool WIFI_OK = false;
bool ledMsgReceivedStatus = HIGH;
unsigned long lastTimeLedMsgReceivedStatusChanged, ledMsgReceivedStatusInterval = 30;

//-------------------------------------------------------------

void setup() {
	Serial.begin(115200);

	pinMode(LED_WIFI_OK, OUTPUT);       // ESP8266 LED - 2, D4, LED_ESP
	pinMode(LED_BATTERY_OK, OUTPUT);   // NodeMCU LED - 16, D0, LED_BUILTIN, BUILTIN_LED
	pinMode(LED_BATTERY_LOW, OUTPUT);   // NodeMCU LED - 16, D0, LED_BUILTIN, BUILTIN_LED
	pinMode(LED_MESSAGE_RECEIVED, OUTPUT);
	
	// turn off all LEDs
	digitalWrite(LED_WIFI_OK, HIGH);
	digitalWrite(LED_BATTERY_OK, LOW);
	digitalWrite(LED_BATTERY_LOW, LOW);
	digitalWrite(LED_MESSAGE_RECEIVED, LOW);

	#ifndef  OLD_VERSION
		checkVoltage();
	#endif // ! OLD_VERSION

#ifdef USE_SOFTAP
	// Create Soft-AP
	// ---------------
	WiFi.mode(WIFI_AP);
	// The network established by softAP will have default IP address of 192.168.4.1. 
	// This address may be changed using softAPConfig: https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/soft-access-point-class.html
	while (WiFi.softAP(ssid, password, softAPChannel, true /*ssid_hidden*/, 4 /*max_connection*/) == false) {
		Serial.println("SoftAP start failed");
		delay(500);
	}

	Serial.println();
	Serial.print("SoftAP started with IP address ");
	Serial.println(WiFi.softAPIP());
#else
	// Connect to WiFi
	// ---------------
	Serial.println("");
	Serial.print("Connecting to ");
	Serial.println(ssid);

	//WiFi.disconnect();				//Prevent connecting to wifi based on previous configuration
	WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
	//WiFi.config(staticIP, subnet, gateway, dns);
	WiFi.mode(WIFI_STA); //WiFi mode station (connect to wifi router only
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

#endif // USE_SOFTAP

	Udp.begin(localPort);
	Serial.print("Listening at port: ");
	Serial.println(Udp.localPort());

	motors.initPins();
	
	lastTimeLedMsgReceivedStatusChanged = millis();
}

//-------------------------------------------------------------

void loop() {

	// turn off the status LED for the "signal sent" status
	if (millis() - lastTimeLedMsgReceivedStatusChanged > ledMsgReceivedStatusInterval)
	{
		ledMsgReceivedStatus = HIGH;
	}
	digitalWrite(LED_MESSAGE_RECEIVED, ledMsgReceivedStatus);


	#ifndef USE_SOFTAP
		// check WiFi-Connection
		if (WiFi.status() != WL_CONNECTED) {
			motors.stop();
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
	#endif // !USE_SOFTAP

	#ifndef  OLD_VERSION
		checkVoltage();
	#endif // ! OLD_VERSION

	// receive and route OSC Messages
	OSCMessage msgIN;
	int size;
	if ((size = Udp.parsePacket()) > 0)
	{
		lastTimeLedMsgReceivedStatusChanged = millis();
		ledMsgReceivedStatus = LOW;

		while (size--) {
			msgIN.fill(Udp.read());
		}
		if (!msgIN.hasError()) {
			msgIN.route("/1/drive", drive);
			msgIN.route("/1/stop", stop);
		}
	}
	delay(20);
}

void checkVoltage()
{
	// check voltage on A0
	int voltage = analogRead(A0);

	if (voltage > 840) // battery good. ca. 11.1V = 3.7V / Cell
	{	
		digitalWrite(LED_BATTERY_OK, HIGH);
		digitalWrite(LED_BATTERY_LOW, LOW);
	}
	else if (voltage > 730) // battery ok'ish. ca. 9.6V = 3.2V / Cell
	{
		digitalWrite(LED_BATTERY_OK, LOW);
		digitalWrite(LED_BATTERY_LOW, HIGH);
	}
	else // deep sleep, battery needs to be changed
	{
		motors.stop();
		Serial.println("Going to deepSleep");
		digitalWrite(LED_BATTERY_OK, LOW);
		for (int i = 0; i < 10; i++) {
			digitalWrite(LED_BATTERY_LOW, HIGH);
			delay(100);
			digitalWrite(LED_BATTERY_LOW, LOW);
			delay(100);
		}
		ESP.deepSleep(0);
	}
}


void stop(OSCMessage &msg, int addrOffset) {
	motors.stop();
}


void drive(OSCMessage &msg, int addrOffset) {
	int duration = 100;
	int m1 = 0;
	int m2 = 0;
	if (msg.isInt(0) && msg.isInt(1)) {
		m1 = msg.getInt(0);
		m2 = msg.getInt(1);
	}
	else
	{
		Serial.println("ERR in drive() - can't get ints from osc parameters");
		motors.stop();
		return;
	}
	Serial.print("Starting Motors:\tm1: ");
	Serial.print(m1);
	Serial.print("\tm2: ");
	Serial.println(m2);


	motors.drive(m1, m2);
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
