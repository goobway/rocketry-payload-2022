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

/* define variables */
#define M_PI 3.14159265358979323846 // pi

float phi;              // system phi
float theta;            // system theta

float phiAcc;           // phi measured from accel.
float thetaAcc;         // theta measured from accel.

float phiGyro = 0;      // phi measured from gyro
float thetaGyro = 0;    // theta measured from gyro

float phiFold = 0;      // phi filtered, old
float phiFnew;          // phi filtered, new
float thetaFold = 0;    // phi filtered, old
float thetaFnew;        // phi filtered, new

float dt;               // change in time since last measurement
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

  /* traking time change, dt in seconds */  
  dt = (millis()-millisOld)/1000.0;
  millisOld=millis();  

  /* calculating measured tilt (XYZ): acceleraometer */
  phiAcc = atan2(acc.y(), acc.z())/2/M_PI*360;    // X (roll), degress
  thetaAcc = atan2(acc.x(), acc.z())/2/M_PI*360;  // Y (pitch), degress

  phi = 0.95*(phi + gyro.x()*dt) + 0.05*phiAcc;
  theta = 0.95*(theta + gyro.y()*dt) + 0.05*thetaAcc;

  /* calculating measured tilt (XYZ): gyroscope */
  thetaGyro = thetaGyro + gyro.y()*dt;            // X (roll), degress
  phiGyro = phiGyro + gyro.x()*dt;                // Y (pitch), degress

  /* low-pass filer on tilt/angle measurements */
  phiFnew = 0.95*phiFold + 0.05*phiAcc;
  thetaFnew = 0.95*thetaFold + 0.05*thetaAcc;

  /* complimentary filter on tilt/angle measurements */
  //phiSys = 0.95*phiGyro + 0.05*phiFnew;
  
  /* print to serial monitor */
  Serial.print(system);     // system calibration number
  Serial.print(",");
  /* acceleration data */
  Serial.print(acc.x());    // accel. in x direction, m/s^2
  Serial.print(",");
  Serial.print(acc.y());    // accel. in y direction, m/s^2
  Serial.print(",");
  Serial.print(acc.z());    // accel. in z direction, m/s^2
  Serial.print(",");
  Serial.print(accCal);     // accel. calibration number
  Serial.print(",");
  /* gyroscope data */
  Serial.print(gyro.x());   // rotational velocity in x direction, m/s
  Serial.print(",");
  Serial.print(gyro.y());   // rotational velocity in y direction, m/s
  Serial.print(",");
  Serial.print(gyro.z());   // rotational velocity in z direction, m/s
  Serial.print(",");
  Serial.print(gyroCal);    // gyro calibration number
  Serial.print(",");
  /* magnetometer data */
  Serial.print(mag.x());    // mag. field in x direction
  Serial.print(",");
  Serial.print(mag.y());    // mag. field in y direction
  Serial.print(",");
  Serial.print(mag.z());    // mag. field in z direction
  Serial.print(",");
  Serial.print(magCal);     // mag. calibration number
  Serial.print(",");
  /* angle data */
  Serial.print(phiAcc);     // phi (roll), accel.
  Serial.print(",");
  Serial.print(thetaAcc);   // theta (pitch), accel.
  Serial.print(",");
  Serial.print(phiGyro);    // phi (roll), gyro
  Serial.print(",");
  Serial.print(thetaGyro);  // theta (pitch) gyro
  Serial.print(",");
  Serial.print(phiFnew);    // phi (roll) low-pass filter, accel.
  Serial.print(",");
  Serial.print(thetaFnew);  // theta (pitch) low-pass filter, accel.
  Serial.print(",");
  Serial.print(phi);        // phi (roll), system
  Serial.print(",");
  Serial.print(theta);      // theta (pitch), system
  Serial.print("\n");

  /* update phi and theta filter values */
  phiFold = phiFnew;
  thetaFold = thetaFnew;

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
