// Name: Jeffrey Matheson
// Language: Arduino
// Project: Rocketry - Payload
// File Description: This file allows you to read both of the BNO055 sensors at the same time
// while also taking their average and outputting these three sets of data to Serial.
// Date: 2/8/2022

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//uint8_t savedOffsets1[22] = {-15, 19, -41, -1, -3, 0, -221, 551, 178, 1000, 620};


Adafruit_BNO055 BNO1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 BNO2 = Adafruit_BNO055(56, 0x29);

void setup() {
  Serial.begin(115200);

  BNO1.begin();
  BNO2.begin();

  delay(100);

  //BNO1.setSensorOffsets(savedOffsets1);
  delay(100);

  //BNO1.setMode(0x0C);
  delay(100);
}


void loop() {
  Serial.println("==============================================================");
  print_accelerometerData(BNO1, '1');
  print_gyroscopeData(BNO1, '1');
  print_magnetometerData(BNO1, '1');
  Serial.println("");
  print_accelerometerData(BNO2, '2');
  print_gyroscopeData(BNO2, '2');
  print_magnetometerData(BNO2, '2');
  Serial.println("");
  printAverage_accelerometerData();
  printAverage_gyroscopeData();
  printAverage_magnetometerData();
  delay(1000);
}

double* get_accelerometerData(Adafruit_BNO055 sensorNum) {
  sensors_event_t accelerometerData;
  sensorNum.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  sensors_event_t* event = &accelerometerData;

  static double xyz[3];
  
  xyz[0] = event->acceleration.x;
  xyz[1] = event->acceleration.y;
  xyz[2] = event->acceleration.z;

  return xyz;
}

double* get_gyroscopeData(Adafruit_BNO055 sensorNum) {
  sensors_event_t gyroscopeData;
  sensorNum.getEvent(&gyroscopeData, Adafruit_BNO055::VECTOR_EULER);

  sensors_event_t* event = &gyroscopeData;

  static double xyz[3];
  
  xyz[0] = event->orientation.x;
  xyz[1] = event->orientation.y;
  xyz[2] = event->orientation.z;

  return xyz;
}

double* get_magnetometerData(Adafruit_BNO055 sensorNum) {
  sensors_event_t magnetometerData;
  sensorNum.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  sensors_event_t* event = &magnetometerData;

  static double xyz[3];
  
  xyz[0] = event->magnetic.x;
  xyz[1] = event->magnetic.y;
  xyz[2] = event->magnetic.z;

  return xyz;
}

void print_accelerometerData(Adafruit_BNO055 sensorNum, char num) {
  double x = get_accelerometerData(sensorNum)[0];
  double y = get_accelerometerData(sensorNum)[1];
  double z = get_accelerometerData(sensorNum)[2];
  
  Serial.print("[");
  Serial.print(num);
  Serial.print("] Accelerometer: ");

  Serial.print("\tX: ");
  Serial.print(x);
  Serial.print("   \tY: ");
  Serial.print(y);
  Serial.print("   \tZ: ");
  Serial.println(z);
}

void print_gyroscopeData(Adafruit_BNO055 sensorNum, char num) {
  double x = get_gyroscopeData(sensorNum)[0];
  double y = get_gyroscopeData(sensorNum)[1];
  double z = get_gyroscopeData(sensorNum)[2];

  Serial.print("[");
  Serial.print(num);
  Serial.print("] Gyroscope: ");

  Serial.print("\t\tX: ");
  Serial.print(x);
  Serial.print("   \tY: ");
  Serial.print(y);
  Serial.print("   \tZ: ");
  Serial.println(z);
}

void print_magnetometerData(Adafruit_BNO055 sensorNum, char num) {
  double x = get_magnetometerData(sensorNum)[0];
  double y = get_magnetometerData(sensorNum)[1];
  double z = get_magnetometerData(sensorNum)[2];

  Serial.print("[");
  Serial.print(num);
  Serial.print("] Magnetometer: ");
  
  Serial.print("\tX: ");
  Serial.print(x);
  Serial.print("   \tY: ");
  Serial.print(y);
  Serial.print("   \tZ: ");
  Serial.println(z);
}

double* getAverage_accelerometerData() {
  double x1 = get_accelerometerData(BNO1)[0];
  double y1 = get_accelerometerData(BNO1)[1];
  double z1 = get_accelerometerData(BNO1)[2];

  double x2 = get_accelerometerData(BNO2)[0];
  double y2 = get_accelerometerData(BNO2)[1];
  double z2 = get_accelerometerData(BNO2)[2];

  static double xyz[3];

  xyz[0] = (x1 + x2) / 2;
  xyz[1] = (y1 + y2) / 2;
  xyz[2] = (z1 + z2) / 2;

  return xyz;
}

double* getAverage_gyroscopeData() {
  double x1 = get_gyroscopeData(BNO1)[0];
  double y1 = get_gyroscopeData(BNO1)[1];
  double z1 = get_gyroscopeData(BNO1)[2];

  double x2 = get_gyroscopeData(BNO2)[0];
  double y2 = get_gyroscopeData(BNO2)[1];
  double z2 = get_gyroscopeData(BNO2)[2];

  static double xyz[3];

  int threshold = 100;

  // If the difference between two of the axis gyroscope readings
  // exceeds that of 'threshold', output the value of x1 instead of their
  // average. This is meant to correct for times where one board reads
  // something like 359 and the other board reads something like 1, which
  // results in an average of ~180 (which is obviously read).
  
  if (abs(x1 - x2) > threshold) {
    xyz[0] = x1;
  } else {
    xyz[0] = (x1 + x2) / 2;
  }

  if (abs(y1 - y2) > threshold) {
    xyz[1] = y1;
  } else {
    xyz[1] = (y1 + y2) / 2;
  }

  if (abs(z1 - z2) > threshold) {
    xyz[2] = z1;
  } else {
    xyz[2] = (z1 + z2) / 2;
  }

  return xyz;
}

double* getAverage_magnetometerData() {
  double x1 = get_magnetometerData(BNO1)[0];
  double y1 = get_magnetometerData(BNO1)[1];
  double z1 = get_magnetometerData(BNO1)[2];

  double x2 = get_magnetometerData(BNO2)[0];
  double y2 = get_magnetometerData(BNO2)[1];
  double z2 = get_magnetometerData(BNO2)[2];

  static double xyz[3];

  xyz[0] = (x1 + x2) / 2;
  xyz[1] = (y1 + y2) / 2;
  xyz[2] = (z1 + z2) / 2;

  return xyz;
}

void printAverage_accelerometerData() {
  double x = getAverage_accelerometerData()[0];
  double y = getAverage_accelerometerData()[1];
  double z = getAverage_accelerometerData()[2];
  
  Serial.print("Avg. Accelerometer: ");

  Serial.print("\tX: ");
  Serial.print(x);
  Serial.print("   \tY: ");
  Serial.print(y);
  Serial.print("   \tZ: ");
  Serial.println(z);
}

void printAverage_gyroscopeData() {
  double x = getAverage_gyroscopeData()[0];
  double y = getAverage_gyroscopeData()[1];
  double z = getAverage_gyroscopeData()[2];

  Serial.print("Avg. Gyroscope: ");

  Serial.print("\tX: ");
  Serial.print(x);
  Serial.print("   \tY: ");
  Serial.print(y);
  Serial.print("   \tZ: ");
  Serial.println(z);
}

void printAverage_magnetometerData() {
  double x = getAverage_magnetometerData()[0];
  double y = getAverage_magnetometerData()[1];
  double z = getAverage_magnetometerData()[2];

  Serial.print("Avg. Magnetometer: ");
  
  Serial.print("\tX: ");
  Serial.print(x);
  Serial.print("   \tY: ");
  Serial.print(y);
  Serial.print("   \tZ: ");
  Serial.println(z);
}
