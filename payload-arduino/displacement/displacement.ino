/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Payload Displacement
   Date: 3/13/2022
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define PI 3.1415926535897932384626433832795 /* pi */

Adafruit_BNO055 bno = Adafruit_BNO055();

double accxOld = 0, accyOld = 0, acczOld = 0; // initial acceleration
double velxOld = 0, velyOld = 0, velzOld = 0; // initial velocity
double disxOld = 0, disyOld = 0, diszOld = 0; // initial displacement
double posxOld = 0, posyOld = 0, poszOld = 0; // initial displacement

double accX, accY, accZ;
double velX, velY, velZ;
double disX, disY, disZ;
double posX, posY, posZ;

int count = 0;

unsigned long dt;
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

  while((system != 3) & (accCal != 3)){
    bno.getCalibration(&system, &gyroCal, &accCal, &magCal);
//    Serial.print("CALIBRATION: Sys=");
//    Serial.print(system, DEC);
//    Serial.print(" Gyro=");
//    Serial.print(gyroCal, DEC);
//    Serial.print(" Accel=");
//    Serial.print(accCal, DEC);
//    Serial.print(" Mag=");
//    Serial.println(magCal, DEC);
    delay(100);
  }
//  Serial.println(""); 
//  Serial.println("Calibrated");
}


void loop(void) {
  /* vector of 3 components saved to "acc" variable, talking to IMU object*/
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* change in time, seconds */
  dt = (millis() - millisOld)/1000.0;
  millisOld = millis();  

  accX = acc.x();
  accY = acc.y();
  accZ = acc.z();

//  velX = accX * dt;
//  velY = accY * dt;
//  velZ = accZ * dt;
//
//  disX = velX * dt;
//  disY = velY * dt;
//  disZ = velZ * dt;
//
//  velX = velxOld + accxOld + ((accX - accxOld) / 2);
//  velY = velyOld + accyOld + ((accY - accyOld) / 2);
//  velZ = velzOld + acczOld + ((accZ - acczOld) / 2);
//
//  disX = disxOld + velxOld + ((velX - velxOld) / 2);
//  disY = disyOld + velyOld + ((velY - velyOld) / 2);
//  disZ = diszOld + velzOld + ((velZ - velzOld) / 2);
//
  velX = (accX - accxOld) * dt;
  velY = (accY - accyOld) * dt;
  velZ = (accZ - acczOld) * dt;

  disX = (velX - velxOld) * dt;
  disY = (velY - velyOld) * dt;
  disZ = (velZ - velzOld) * dt;

  posX = disX + posxOld;
  posY = disY + posyOld;
  posZ = disZ + poszOld;

  /* print data to serial */
//  Serial.print(accX);
//  Serial.print(", ");
//  Serial.print(accY);
//  Serial.print(", ");
//  Serial.print(accZ);
//  Serial.print(", ");
//  Serial.print(velX);
//  Serial.print(", ");
//  Serial.print(velY);
//  Serial.print(", ");
//  Serial.print(velZ);
//  Serial.print(", "); 
  Serial.print(millisOld);
//  Serial.print(", ");
//  Serial.print(dt, 6);  
  Serial.print(", ");    
  Serial.print(accX, 6);
  Serial.print(", ");
  Serial.print(accY, 6);
  Serial.print(", ");
  Serial.print(accZ, 6);
  Serial.print(", ");    
  Serial.print(euler.x(), 6);
  Serial.print(", ");
  Serial.print(euler.y(), 6);
  Serial.print(", ");
  Serial.print(euler.z(), 6);
  Serial.print(", ");    
  Serial.print(gyro.x(), 6);
  Serial.print(", ");
  Serial.print(gyro.y(), 6);
  Serial.print(", ");
  Serial.println(gyro.z(), 6);

  /* update values */
  accxOld = accX;
  accyOld = accY;
  acczOld = accZ;

  velxOld = velX;
  velyOld = velY;
  velzOld = velZ;

  disxOld = disX;
  disyOld = disY;
  diszOld = disZ;

  posxOld = posX;
  posyOld = posY;
  poszOld = posZ;

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
