#include <Arduino.h>
#include <Wire.h>
#include <7Semi_BNO055.h>

BNO055_7Semi imu = BNO055_7Semi();



// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  imu.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  printf("hello world");
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}