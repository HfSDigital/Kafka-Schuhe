/*
 Name:		Kafka_Sender.ino
 Created:	8/19/2019 10:32:55 AM
 Author:	Puppe
*/

#define USE_SOFTAP

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include "pw.h"

#define LED_ESP 2

#define JOYSTICK_AXIS_X A0
#define JOYSTICK_AXIS_Y A3

#define POTI_MAXSPEED A6
#define POTI_FORWARD_SPEED A7
#define POTI_STEERING_ADJUST A4
#define POTI_NOT_USED A5

#define JOYTICK_BTN A15

// OSC-Stuff ----------------------------------------------------------
IPAddress outIp;
WiFiUDP Udp;
const unsigned int localPort = 9000;
const unsigned int destPort = 8000;


bool isRunning = false;
uint16_t centerXAxis = 0;
uint16_t centerYAxis = 0;

float tolerance = 0.01;
bool isCalibrated = false;
bool reverseGear = false;
//-------------------------------------------------------------

void setup() {

	Serial.begin(115200);

	pinMode(LED_ESP, OUTPUT);       // ESP8266 LED - 2, D4, LED_ESP
	pinMode(LED_BUILTIN, OUTPUT);   // NodeMCU LED - 16, D0, LED_BUILTIN, BUILTIN_LED

	digitalWrite(LED_ESP, HIGH);	// Turn ESP's tiny LED off at startup (anyway, that pin is used for the tb6612fng
	digitalWrite(LED_BUILTIN, LOW);	// turn IR and Control-LED ON at startup!
	

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
	if (!outIp.fromString(softAPIP))
	{
		Serial.println("can't parse Remote-IP-Address.");
	}
	else {
		Serial.print("outIp: ");
		Serial.println(outIp.toString());
	}
}

//-------------------------------------------------------------

void loop() {
	
	// if we drive with forward speed, everything else will be ignored right now...
	float forwardSpeed = analogRead(POTI_FORWARD_SPEED);		// for driving both motors with the same speed just forward. for testing and debuging
	forwardSpeed = map(forwardSpeed, 0, 4095, 0, 100);

	if (forwardSpeed > 0)
	{
		drive(forwardSpeed, forwardSpeed);

		delay(50);
		return;
	}


	// ...driving with joystick:
	if (!isCalibrated)
	{
		delay(3000);
		centerXAxis = analogRead(JOYSTICK_AXIS_X);
		centerYAxis = analogRead(JOYSTICK_AXIS_Y);
		Serial.print("cx: ");
		Serial.print(centerXAxis);
		Serial.print("\tcy: ");
		Serial.println(centerYAxis);
		isCalibrated = true;
	}

	float xAxis = 0, yAxis = 0;

	float xA = analogRead(JOYSTICK_AXIS_X);
	float yA = analogRead(JOYSTICK_AXIS_Y);
	float exp = mapf(analogRead(POTI_NOT_USED), 0, 4095, 1.0, 4.0);

	xAxis = (xA > centerXAxis) ? mapf(xA, centerXAxis, 4095, 0, 1.0) : mapf(xA, centerXAxis, 0, 0, -1.0);
	yAxis = (yA > centerYAxis) ? mapf(yA, centerYAxis, 4095, 0, 1.0) : mapf(yA, centerYAxis, 0, 0, -1.0);

	xAxis *= -1; // reverse the steering
	reverseGear = yAxis > 0;

	//xAxis = powSigned(xAxis, exp);
	//yAxis = powSigned(yAxis, exp);	// only power the steering

	//Serial.print("x: ");
	//Serial.print(xA);
	//Serial.print("\t\ty: ");
	//Serial.print(yA);

	//Serial.print("\t\tx-c: ");
	//Serial.print(xA - centerXAxis);
	//Serial.print("\t\ty-c: ");
	//Serial.print(yA - centerYAxis);


	if ((absf(yAxis) < tolerance) && (absf(xAxis) < tolerance) && isRunning) {
		stop();
	}
	else if ((absf(yAxis) > tolerance) || (absf(xAxis) > tolerance)) {
		float rotationPower = 1.0;
		float exp = mapf(analogRead(POTI_NOT_USED), 0, 4095, 1.0, 4.0);
		float d1Clamp = powSigned( mapf(constrain(xAxis, -1.0,   0), -1.0,   0, rotationPower, 0), exp);
		float d2Clamp = powSigned( mapf(constrain(xAxis,    0, 1.0),    0, 1.0, 0, rotationPower), exp);

		float m1 = constrain(yAxis - d1Clamp + d2Clamp, -1.0, 1.0);
		float m2 = constrain(yAxis - d2Clamp + d1Clamp, -1.0, 1.0);

		Serial.print("\t\tx: ");
		Serial.println(xAxis);
		//Serial.print("\t\ty: ");
		//Serial.print(yAxis);

		Serial.print("\t\texp: ");
		Serial.println(exp);

		//Serial.print("\t\tm1: ");
		//Serial.print(m1);
		//Serial.print("\t\tm2: ");
		//Serial.println(m2);

		float maxSpeed = analogRead(POTI_MAXSPEED);
		maxSpeed = mapf(maxSpeed, 0, 4095, 0, 100);
		m1 = mapf(m1, -1.0, 1.0, -maxSpeed, maxSpeed);
		m2 = mapf(m2, -1.0, 1.0, -maxSpeed, maxSpeed);

		drive(int(m2), int(m1));
	}



	delay(20);
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
	int steeringAdjust = analogRead(POTI_STEERING_ADJUST);
	steeringAdjust = map(steeringAdjust, 0, 4095, -25, 25);

	if (reverseGear) steeringAdjust *= -1;
	
	m1 -= steeringAdjust;
	m2 += steeringAdjust;
	
	m1 = constrain(m1, -100, 100);
	m2 = constrain(m2, -100, 100);

	//Serial.print("steeringAdjust: ");
	//Serial.print(steeringAdjust);
	//Serial.print("\tm1: ");
	//Serial.print(m1);
	//Serial.print("\tm2: ");
	//Serial.println(m2);
	Serial.print(m1);
	Serial.print(",");
	Serial.println(m2);

	digitalWrite(LED_ESP, LOW);
	
	OSCMessage msg("/1/drive");
	msg.add(m1);
	msg.add(m2);
	Udp.beginPacket(outIp, destPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();

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