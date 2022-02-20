/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Compass
   Date: 2/7/2022
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Cardinal.h>

/* define variables */
#define M_PI 3.14159265358979323846 // pi

Cardinal cardinal;
int cardinal_integer;

float phi;              // system roll, phi
float theta;            // system pitch, theta
float psi;              // system yaw, psi

float phiRad;           // phi in radians
float thetaRad;         // theta in radians

float phiAcc;           // phi measured from accel.
float thetaAcc;         // theta measured from accel.

float phiGyro = 0;      // phi measured from gyro
float thetaGyro = 0;    // theta measured from gyro

float xMag;
float yMag;

float xMag_euler;
float yMag_euler;

float phiFold = 0;      // phi filtered, old
float phiFnew;          // phi filtered, new
float thetaFold = 0;    // phi filtered, old
float thetaFnew;        // phi filtered, new

float dt;
unsigned long millisOld;

/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

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
}

void loop() {
  /* calibration: 0 = least calibrated, 3 = most calibrated */
  uint8_t system, accCal, gyroCal, magCal = 0;
  bno.getCalibration(&system, &accCal, &gyroCal, &magCal);
  
  /* vector of 3 components saved to "acc" variable, talking to IMU object*/
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  /* vector of 3 components saved to "gyro" variable, talking to IMU object*/
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  /* vector of 3 components saved to "mag" variable, talking to IMU object*/
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  /* vector of 3 components saved to "euler" variable, talking to IMU object*/
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* traking time change, dt in seconds */  
  dt = (millis()-millisOld)/1000.0;
  millisOld = millis();  

  /* calculating measured tilt (XY): acceleraometer */
  phiAcc = atan2(acc.y()/9.8, acc.z()/9.8)*180/M_PI;    // X (roll), degress
  thetaAcc = atan2(acc.x()/9.8, acc.z()/9.8)*180/M_PI;  // Y (pitch), degress

  /* complimentary filter on tilt/angle measurements */
  phi = 0.95*(phi + gyro.x()*dt) + 0.05*phiAcc;
  theta = 0.95*(theta + gyro.y()*dt) + 0.05*thetaAcc;

  /* calculating measured tilt (XY): gyroscope */
  thetaGyro = thetaGyro + gyro.y()*dt;            // X (roll), degress
  phiGyro = phiGyro + gyro.x()*dt;                // Y (pitch), degress

  /* calculating measured tilt: magnetometer */
  phiRad = phi/360*(2*M_PI);
  thetaRad = theta/360*(2*M_PI);
  
  xMag = mag.x()*cos(theta) - mag.y()*sin(phi)*sin(theta) + mag.z()*cos(phi)*sin(theta);
  yMag = mag.y()*cos(phi) + mag.z()*sin(phi);
  psi = atan2(yMag, xMag)*180/M_PI;
  while(psi < 0) {
    psi += 360;
  }

  //xMag_euler = mag.x()*cos(euler.x()) - mag.y()*sin(euler.y())*sin(euler.x()) + mag.z()*cos(euler.y())*sin(euler.x());
  //yMag_euler = mag.y()*cos(euler.y()) + mag.z()*sin(euler.x());
  //psi = atan2(yMag_euler, xMag_euler)*180/M_PI;


  /* low-pass filer on tilt/angle measurements */
  phiFnew = 0.95*phiFold + 0.05*phiAcc;
  thetaFnew = 0.95*thetaFold + 0.05*thetaAcc;

  cardinal_integer = cardinal.getInteger(3, psi);
  
  /* display data */
  Serial.print(cardinal.getString(3, psi));
  Serial.print(",");
  
  /* print to serial monitor */
//  Serial.print(system);     // system calibration number
//  Serial.print(",");
  /* acceleration data */
//  Serial.print(acc.x());    // accel. in x direction, m/s^2
//  Serial.print(",");
//  Serial.print(acc.y());    // accel. in y direction, m/s^2
//  Serial.print(",");
//  Serial.print(acc.z());    // accel. in z direction, m/s^2
//  Serial.print(",");
//  Serial.print(accCal);     // accel. calibration number
//  Serial.print(",");
  /* gyroscope data */
//  Serial.print(gyro.x());   // rotational velocity in x direction, m/s
//  Serial.print(",");
//  Serial.print(gyro.y());   // rotational velocity in y direction, m/s
//  Serial.print(",");
//  Serial.print(gyro.z());   // rotational velocity in z direction, m/s
//  Serial.print(",");
//  Serial.print(gyroCal);    // gyro calibration number
//  Serial.print(",");
  /* magnetometer data */
//  Serial.print(mag.x());    // mag. field in x direction
//  Serial.print(",");
//  Serial.print(mag.y());    // mag. field in y direction
//  Serial.print(",");
//  Serial.print(mag.z());    // mag. field in z direction
//  Serial.print(",");
  Serial.print(magCal);     // mag. calibration number
  Serial.print(",");
  /* angle data */
//  Serial.print(phiAcc);     // phi (roll), accel.
//  Serial.print(",");
//  Serial.print(thetaAcc);   // theta (pitch), accel.
//  Serial.print(",");
//  Serial.print(phiGyro);    // phi (roll), gyro
//  Serial.print(",");
//  Serial.print(thetaGyro);  // theta (pitch) gyro
//  Serial.print(",");
//  Serial.print(phiFnew);    // phi (roll) low-pass filter, accel.
//  Serial.print(",");
//  Serial.println(thetaFnew);  // theta (pitch) low-pass filter, accel.
//  Serial.print(",");
//  Serial.print(phi);        // phi (roll), system
//  Serial.print(",");
//  Serial.print(theta);      // theta (pitch), system
//  Serial.print(",");
    Serial.println(psi);        // psi (yaw), system
//  Serial.print("\n");

  /* update phi and theta filter values */
  phiFold = phiFnew;
  thetaFold = thetaFnew;

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
