/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Compass
   Date: 2/18/2022
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Cardinal.h>

/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define M_PI 3.14159265358979323846 // pi

Adafruit_BNO055 bno = Adafruit_BNO055();


Cardinal cardinal;
int cardinal_integer;
double yaw;

void setup() {
  // put your setup code here, to run once:
  /* initialize BNO055 */
  Serial.begin(115200);
  bno.begin();
  delay(1000);
  
  /* use crystal reference on board (not chip) */
  bno.setExtCrystalUse(true);
  
  /* temperature reading */
  int8_t temp = bno.getTemp();
}

void loop() {
  // put your main code here, to run repeatedly:
  /* calibration: 0 = least calibrated, 3 = most calibrated */
  uint8_t system, accCal, gyroCal, magCal = 0;
  bno.getCalibration(&system, &accCal, &gyroCal, &magCal);
  /* vector of 3 components saved to "euler" variable, talking to IMU object*/
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* vector of 3 components saved to "mag" variable, talking to IMU object*/
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  yaw = atan2(mag.y(), mag.x())*180/M_PI;
  while(yaw < 0) {
    yaw += 360;
  }

  cardinal_integer = cardinal.getInteger(3, yaw);

  /* display data */
  Serial.print(magCal);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");  
  Serial.println(cardinal.getString(3, yaw));
}
