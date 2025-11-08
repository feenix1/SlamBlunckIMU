#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 imuSensor = Adafruit_BNO055();

float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() {
  Serial.begin(9600);
  imuSensor.begin();
}

void loop() {
  imuSensor.printSensorDetails();
}