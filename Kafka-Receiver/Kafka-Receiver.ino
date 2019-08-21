/*
 Name:		Kafka_Receiver.ino
 Created:	8/19/2019 10:24:52 AM
 Author:	Puppe
*/

/*

  - reset WiFi SSID and Password: press RST-Button shortly, release and press FLASH-Button until
	the onboard-LED blinks fast. This will erase EEPROM.

*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include "pw.h"
#include "TB6612FNG.h"

#define LED_ESP D4



// OSC-Stuff ----------------------------------------------------------
IPAddress outIp;
WiFiUDP Udp;
const unsigned int localPort = 8000;
const unsigned int destPort = 9000;



// Motor -------------------------------------------------------------
tb6612fng motors;




//-------------------------------------------------------------

void setup() {
	Serial.begin(115200);

	pinMode(LED_ESP, OUTPUT);       // ESP8266 LED - 2, D4, LED_ESP
	pinMode(LED_BUILTIN, OUTPUT);   // NodeMCU LED - 16, D0, LED_BUILTIN, BUILTIN_LED

	digitalWrite(LED_ESP, HIGH);	// Turn ESP's tiny LED off at startup (anyway, that pin is used for the tb6612fng
	digitalWrite(LED_BUILTIN, LOW);	// turn IR and Control-LED ON at startup!

	// Create Soft-AP
	// ---------------
	WiFi.mode(WIFI_AP);
	// The network established by softAP will have default IP address of 192.168.4.1. 
	// This address may be changed using softAPConfig: https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/soft-access-point-class.html
	while (WiFi.softAP(ssid, password, softAPChannel, true /*ssid_hidden*/, 1 /*max_connection*/) == false) {
		Serial.println("SoftAP start failed");
		delay(500);
	}
	
	Udp.begin(localPort);
	Serial.println();
	Serial.print("SoftAP started with IP address ");
	Serial.println(WiFi.softAPIP());
	Serial.print("Listening at port: ");
	Serial.println(Udp.localPort());
	
	Serial.print("local IP address: ");
	Serial.println(WiFi.localIP());

	motors.initPins();
}

//-------------------------------------------------------------

void loop() {
	OSCMessage msgIN;
	int size;
	if ((size = Udp.parsePacket()) > 0)
	{
		while (size--) {
			msgIN.fill(Udp.read());
		}
		if (!msgIN.hasError()) {
			//msgIN.route("/pair/request", pair);
			//msgIN.route("/1/toggleLED", toggleOnOff);
			//msgIN.route("/1/firstStep", firstStep);
			msgIN.route("/1/drive", drive);
			msgIN.route("/1/stop", stop);
		}
	}


	//if (!outIp.fromString("192.168.4.2"))
	//{
	//	Serial.println("can't parse IP-Address. Pairing failed.");
	//	return;
	//}

	//OSCMessage msg("/pair/accept");
	//msg.add("test");
	//Udp.beginPacket(outIp, destPort);
	//msg.send(Udp);
	//Udp.endPacket();
	//msg.empty();

	//delay(500);
}

//-------------------------------------------------------------

void toggleOnOff(OSCMessage &msg, int addrOffset) {
	int ledState = (boolean)msg.getInt(0);
	Serial.print("LED_State: ");
	Serial.println(ledState);
	digitalWrite(LED_BUILTIN, !ledState);
	//digitalWrite(LED_ESP, ledState);
	//digitalWrite(LED_IR, ledState);
}

/*-------------------------------------------------------------
void pair(OSCMessage &msg, int addrOffset);
/*-------------------------------------------------------------
This function is called when the remote computer wants to pair. To do that,
the remote computer needs to broadcast an OSC-Message to the OSC-address /pair/request

The OSC-Message should contain the remote IP as a string in the format
"192.168.12.123". If the IP gets parsed correctly, the
client now knows the remote computers IP and can answer. (I didn't find a function
to read the remote's IP from the OSC-Message, so I send it as a string)

The answer goes to the address /pair/accept and contains the
human readable name of the shoe as a string in the first argument.
/*-------------------------------------------------------------*/
//void pair(OSCMessage &msg, int addrOffset) {
//	Serial.println("pair");
//	if (msg.isString(0)) {
//		int length = msg.getDataLength(0);
//		char remoteIP[length];
//		msg.getString(0, remoteIP, length);
//
//		if (!outIp.fromString(remoteIP))
//		{
//			Serial.println("can't parse IP-Address. Pairing failed.");
//			return;
//		}
//
//		OSCMessage msg("/pair/accept");
//		msg.add(shoeName);
//		Udp.beginPacket(outIp, destPort);
//		msg.send(Udp);
//		Udp.endPacket();
//		msg.empty();
//	}
//}

//
//void firstStep(OSCMessage &msg, int addrOffset) {
//	int duration = 100;
//	int speed = 50;
//	if (msg.isInt(0) && msg.isInt(1)) {
//		duration = msg.getInt(0);
//		speed = msg.getInt(1);
//		Serial.print("duration to drive: ");
//	}
//	else
//	{
//		Serial.print("duration to drive set to default: ");
//	}
//	Serial.println(duration);
//	Serial.println(speed);
//
//
//	motors.drive(speed, speed);
//	delay(duration);
//	motors.stop();
//}


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

