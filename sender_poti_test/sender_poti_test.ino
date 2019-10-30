#include "oneEuroFilter.h"
#include "averageFilter.h"

#define JOYSTICK_AXIS_X A0  // xAxis: throttle
#define JOYSTICK_AXIS_Y A3  // yAxis: steering
float minX = 4096, maxX = 0;
float minY = 4096, maxY = 0;

OneEuroFilter fX(0.01/*frequency*/, 0.001/*mincutoff*/, 0.001 /*beta*/, 0.001/*dcutoff*/);
averageFilter avX;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float xA = analogRead(JOYSTICK_AXIS_X); // xAxis: throttle
  float yA = analogRead(JOYSTICK_AXIS_Y); // yAxis: steering
  if (xA > maxX)
    maxX = xA;
  else if (xA < minX)
    minX = xA;

  if (yA > maxY)
    maxY = yA;
  else if (yA < minY)
    minY = yA;

  Serial.print(xA);
  Serial.print(",");

  float avfiltered = avX.filter(xA);
  Serial.print(avfiltered);
  Serial.print(",");
  /*
  Serial.print(minX);
  Serial.print(",");
  
  Serial.print(maxX);
  Serial.print(",");

    Serial.print(minY);
    Serial.print(",");
    Serial.print(maxY);
    Serial.print(",");
    */
  float filtered = fX.filter(xA, millis());
  Serial.println(filtered);
  //Serial.println(yA);
  delay(20);
}
