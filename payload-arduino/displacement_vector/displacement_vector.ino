/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Payload
   Date: 2/19/2022
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define PI 3.1415926535897932384626433832795 /* pi */

Adafruit_BNO055 bno = Adafruit_BNO055();

double accelerationX = 0;
double accelerationY = 0;
double accelerationZ = 0;

double acc_mag;

double velocityX = 0, velocityY = 0;
double vxOld, vyOld;
double dvX, dvY;

double positionX, positionY;
double posxOld, posyOld;
double deltaX, deltaY;

double vel_x_filter_new, vel_y_filter_new;
double vel_x_filter_old = 0, vel_y_filter_old = 0;

double pos_x_filter_new, pos_y_filter_new;
double pos_x_filter_old = 0, pos_y_filter_old = 0;

double dt;
unsigned long millisOld;

void setup() {
  /* initialize BNO055 */
  Serial.begin(115200);
  bno.begin();
  delay(1000);
  
  /* use crystal reference on board (not chip) */
  bno.setExtCrystalUse(true);
  millisOld = millis();
  
  /* temperature reading */
  int8_t temp = bno.getTemp();

  /* calibration: 0 = least calibrated, 3 = most calibrated */
  uint8_t system, accCal, gyroCal, magCal = 0;

  while(system != 3){
    bno.getCalibration(&system, &gyroCal, &accCal, &magCal);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyroCal, DEC);
    Serial.print(" Accel=");
    Serial.print(accCal, DEC);
    Serial.print(" Mag=");
    Serial.println(magCal, DEC);
    delay(100);
  }
  Serial.println(""); 
  Serial.println("Calibrated");
}


void loop(void) {

  /* vector of 3 components saved to "euler" variable, talking to IMU object*/
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* vector of 3 components saved to "acc" variable, talking to IMU object*/
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* traking time change, dt in seconds */  
  dt = (millis()-millisOld)/1000.0;
  millisOld = millis();  

  accelerationX = acc.x();
  accelerationY = acc.y();
  accelerationZ = acc.z();

  acc_mag = abs(accelerationX) + abs(accelerationY) + abs(accelerationZ);
  
  /****** finding position on 2D plane, meters ******/
  
  /* velocity x direction */
  dvX = accelerationX * dt;
  velocityX = vxOld + dvX;
    
  /* velocity y direction */
  dvY = accelerationY * dt;
  velocityY = vyOld + dvY;

  /* position x direction */
  deltaX = velocityX * dt;
  positionX = posxOld + deltaX;
  
  /* position y direction */
  deltaY = velocityY * dt;
  positionY = posyOld + deltaY;

  /* low-pass filter of measurements */
  vel_x_filter_new = 0.95*vel_x_filter_old + 0.05*velocityX;
  vel_y_filter_new = 0.95*vel_y_filter_old + 0.05*velocityY;
  pos_x_filter_new = 0.95*pos_x_filter_old + 0.05*positionX;
  pos_y_filter_new = 0.95*pos_y_filter_old + 0.05*positionY;

  /* display data */
  Serial.print(accelerationX);
  Serial.print(", ");
  Serial.print(accelerationY);
  Serial.print(", ");
  Serial.println(accelerationZ);
//  Serial.print(vel_x_filter_old);
//  Serial.print(", ");
//  Serial.print(vel_y_filter_old);
//  Serial.print(", ");  
//  Serial.print(pos_x_filter_new);
//  Serial.print(", ");
//  Serial.print(pos_y_filter_new);

  /* update filter values */
  pos_x_filter_old = pos_x_filter_new;
  pos_y_filter_old = pos_y_filter_new;
  vel_x_filter_old = vel_x_filter_new;
  vel_y_filter_old = vel_y_filter_new;

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
