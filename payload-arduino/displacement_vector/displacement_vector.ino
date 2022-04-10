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
#include <CSV_Parser.h>

/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define PI 3.1415926535897932384626433832795 /* pi */

Adafruit_BNO055 bno = Adafruit_BNO055();

vector<double> accX;
double accY = 0;
double accZ = 0;

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
}


void loop(void) {

  /* vector of 3 components saved to "euler" variable, talking to IMU object*/
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* vector of 3 components saved to "acc" variable, talking to IMU object*/
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* traking time change, dt in seconds */  
  dt = (millis()-millisOld)/1000.0;
  millisOld = millis();  

  accX = acc.x();
  accY = acc.y();
  accZ = acc.z();


  /****** finding velocity, m/s ******/

  /* velocity x direction */
  dvX = accelerationX * dt;
  velocityX = vxOld + dvX;
    
  /* velocity y direction */
  dvY = accelerationY * dt;
  velocityY = vyOld + dvY;

  /* velocity z direction */
  dvZ = accelerationZ * dt;
  velocityZ = vzOld + dvZ;
  
  /****** finding displacement, meters ******/

  /* position x direction */
  deltaX = velocityX * dt;
  positionX = posxOld + deltaX;
  
  /* position y direction */
  deltaY = velocityY * dt;
  positionY = posyOld + deltaY;

  /* position z direction */
  deltaZ = velocityZ * dt;
  positionZ = poszOld + deltaZ;

  /* display data */
  Serial.print(accelerationX);
  Serial.print(", ");
  Serial.print(accelerationY);
  Serial.print(", ");
  Serial.println(accelerationZ);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
