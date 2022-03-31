/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Payload
   Desription: Conversion of final displacement to grid coordinate
   Date: 3/23/2022
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define PI 3.1415926535897932384626433832795 /* pi */

Adafruit_BNO055 bno = Adafruit_BNO055();

double offsetX = 250;       // value from xBee, feet
double offsetY = 500;       // value from xBee, feet
int startX = 10;         // feet
int startY = 10;         // feet
double dispX = 3000;    // meters
double dispY = 3000;    // meters
int endX;
int endY;
String startCoord = "J10";
String finalCoord;

char letters[20] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'};

void setup() {
  Serial.begin(115200);
  delay(1000);
  getFinalCoord();
}

void loop() {
  // none
}

void getFinalCoord(){
  // 1. Fetch starting coordinate (center = J10)
  Serial.print("Starting grid coordinate: ");
  getStartCoord();
  Serial.println(startCoord);
  
  // 2. Wait for displacement calculations to finish...
  Serial.println("Calculating displacement...");
  // findDisplacement();
  Serial.println("");
  delay(1000);
  
  // 3. Convert final displacement from meters to feet
  Serial.println("Converting to feet...");
  dispX = dispX * 3.281;
  dispY = dispY * 3.281;
  
  Serial.print("X (ft): ");
  Serial.println(dispX);
  Serial.print("Y (ft): ");
  Serial.println(dispY);
  Serial.println("");
  
  delay(1000);  
  
  // 4. Find new grid coordinate based on start
  endX = round(startX + (dispX / 250));
  endY = round(startY + (dispY / 250));
  finalCoord = letters[endX] + endY;

  return finalCoord;
}

void getStartCoord() {
  offsetX = offsetX / 250;
  offsetY = offsetY / 250;
  startX = round(startX + offsetX);
  startY = round(startY + offsetY);
  startCoord = letters[startX] + startY;
  
  return startCoord;
}
