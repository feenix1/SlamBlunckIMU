#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <7Semi_BNO055.h> 
#include <SD.h> 

// #define ENABLE_SERIAL


// PIN DEFS
const int chipSelect = 2;

// GLOBAL IMU
BNO055_7Semi imuSensor = BNO055_7Semi();

void error() {
  SPI.end();
  pinMode(LED_BUILTIN, OUTPUT);
  while(true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

File dataFile;

// Schema
String header = "timestamp,accelX,accelY,accelZ,gravX,gravY,gravZ,angVelX,angVelY,angVelZ,pitch,roll,heading,w,x,y,z,temprature,sysCal,gyroCal,accelCal,magCal";

void setup() {
  
  #ifdef ENABLE_SERIAL
  Serial.begin(9600);
  #endif

  if (!SD.begin(chipSelect)) {
    #ifdef ENABLE_SERIAL
    Serial.println("SD start error");
    #endif
    error();
  }

  if (!imuSensor.begin()) {
    #ifdef ENABLE_SERIAL
    Serial.println("IMU start error");
    #endif
    error();
  }
  
  imuSensor.setMode(Mode::NDOF);
  imuSensor.setAxis(0x24, 0x00); // default
  
  dataFile = SD.open("log.txt", FILE_WRITE);
  dataFile.println("\n=== CONTROLLER RESET =========================================\n");
  dataFile.println(header);
  dataFile.close();
}
 

void loop() {
  dataFile = SD.open("log.txt", FILE_WRITE);
  if (!dataFile) {
    #ifdef ENABLE_SERIAL
    Serial.println("Could not open file");
    #endif
    error();
  }
  
  unsigned long timestamp = millis();

  // X -left/+right, Y -back/+forward, Z -down,+up

  int16_t accelX, accelY, accelZ; // m/s^2
  // Datasheet says not useable to integrate position/velocity
  // int16_t linAccelX, linAccelY, linAccelZ;
  int16_t gravX, gravY, gravZ; // m/s^2
  // Doesn't seem to be reading at all
  // int16_t magX, magY, magZ; // micro Tesla
  int16_t angVelX, angVelY, angVelZ;
  float pitch, roll, heading;
  float w, x, y, z;
  int8_t temprature;

  imuSensor.readAccel(accelX, accelY, accelZ);
  // imuSensor.readLinear(linAccelX, linAccelY, linAccelZ);
  imuSensor.readGravity(gravX, gravY, gravZ);
  // imuSensor.readMag(magX, magY, magZ);
  imuSensor.readGyro(angVelX, angVelY, angVelZ);
  imuSensor.readEuler(heading, roll, pitch);
  imuSensor.readQuat(w, x, y, z);
  temprature = imuSensor.temperatureC();

  uint8_t sysCal;
  uint8_t gyroCal;
  uint8_t accelCal;
  uint8_t magCal;
  imuSensor.calibBreakdown(sysCal, gyroCal, accelCal, magCal);
  
  String lineOut = "";
  
  lineOut += String(timestamp) + ",";

  // lineOut += "Acceleration: ";
  lineOut += String(accelX) + ","; 
  lineOut += String(accelY) + ","; 
  lineOut += String(accelZ) + ","; 

  lineOut += String(gravX) + ","; 
  lineOut += String(gravY) + ","; 
  lineOut += String(gravZ) + ","; 

  // lineOut += String(magX) + ", "; 
  // lineOut += String(magY) + ", "; 
  // lineOut += String(magZ) + ", "; 

  lineOut += String(angVelX) + ","; 
  lineOut += String(angVelY) + ","; 
  lineOut += String(angVelZ) + ","; 

  // lineOut += "Orientation: ";
  lineOut += String(pitch) + ","; 
  lineOut += String(roll) + ","; 
  lineOut += String(heading) + ",";

  lineOut += String(w) + ","; 
  lineOut += String(x) + ","; 
  lineOut += String(y) + ",";
  lineOut += String(z) + ",";

  lineOut += String(temprature) + ","; 

  // lineOut += "Calibrated: "; 
  lineOut += String(sysCal) + ",";
  lineOut += String(gyroCal) + ",";
  lineOut += String(accelCal) + ",";
  lineOut += String(magCal) + "";

  #ifdef ENABLE_SERIAL
  Serial.println(lineOut);
  #endif
  
  dataFile.println(lineOut);
  dataFile.close();
}
