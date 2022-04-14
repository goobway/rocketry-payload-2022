/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Payload Displacement
   Date: 3/13/2022
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_Rocketry.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>

// Operation Modes
#define OPERATION_MODE_CONFIG         0x00
#define OPERATION_MODE_ACCONLY        0x01
#define OPERATION_MODE_MAGONLY        0x02
#define OPERATION_MODE_GYRONLY        0x03
#define OPERATION_MODE_ACCMAG         0x04
#define OPERATION_MODE_ACCGYRO        0x05
#define OPERATION_MODE_MAGGYRO        0x06
#define OPERATION_MODE_AMG            0x07
#define OPERATION_MODE_IMUPLUS        0x08
#define OPERATION_MODE_COMPASS        0x09
#define OPERATION_MODE_M4G            0x0A
#define OPERATION_MODE_NDOF_FMC_OFF   0x0B
#define OPERATION_MODE_NDOF           0x0C


/* set time delay between samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)
#define PI 3.1415926535897932384626433832795 /* pi */

Adafruit_BNO055_Rocketry bno = Adafruit_BNO055_Rocketry();

unsigned long timestamp = millis();

void setup() {
  /* initialize BNO055 */
  Serial.begin(9600);
  bno.begin();
  delay(1000);

  bno.setMode(OPERATION_MODE_NDOF);     // Set the Operation Mode of the sensor to Accelerometer, Magnetometer, and Gyroscope
  Serial.println("BNO IN NDOF MODE");
  delay(50);
  //  bno.setGRange(0x0F);                 // Set the sensor's G Range to 16Gs
  //  delay(50);

  /* use crystal reference on board (not chip) */
  bno.setExtCrystalUse(true);

  /* temperature reading */
  int8_t temp = bno.getTemp();

  /* calibration: 0 = least calibrated, 3 = most calibrated */
  uint8_t system, accCal, gyroCal, magCal = 0;

  while ((accCal != 3) && (system != 3)) {
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

  int i = 0;
  while (i < 10) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055_Rocketry::VECTOR_EULER);
    Serial.print(euler.x(), 6);
    Serial.print(", ");
    Serial.print(euler.y(), 6);
    Serial.print(", ");
    Serial.println(euler.z(), 6);
    i++;
  }


  bno.setMode(OPERATION_MODE_AMG);     // Set the Operation Mode of the sensor to Accelerometer, Magnetometer, and Gyroscope
  delay(50);
  bno.setGRange(0x0F);                 // Set the sensor's G Range to 16Gs
  delay(50);

}


void loop(void) {
  /* vector of 3 components saved to "acc" variable, talking to IMU object*/
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055_Rocketry::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055_Rocketry::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055_Rocketry::VECTOR_GYROSCOPE);
  imu::Vector<3> acclin = bno.getVector(Adafruit_BNO055_Rocketry::VECTOR_LINEARACCEL);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055_Rocketry::VECTOR_GRAVITY);

  imu::Quaternion quat = bno.getQuat();
  //imu::Quaternion Adafruit_BNO055::getQuat()


  /* print data to serial */
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(acc.x(), 6);
  Serial.print(", ");
  Serial.print(acc.y(), 6);
  Serial.print(", ");
  Serial.print(acc.z(), 6);
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
  Serial.print(gyro.z(), 6);
  Serial.print(", ");
  Serial.print(acclin.x(), 6);
  Serial.print(", ");
  Serial.print(acclin.y(), 6);
  Serial.print(", ");
  Serial.println(acclin.z(), 6);
  //  Serial.print(", ");
  //  Serial.print(grav.x(), 6);
  //  Serial.print(", ");
  //  Serial.print(grav.y(), 6);
  //  Serial.print(", ");
  //  Serial.print(grav.z(), 6);
  //  Serial.print(", ");
  //  Serial.print(quat.w(), 6);
  //  Serial.print(", ");
  //  Serial.print(quat.x(), 6);
  //  Serial.print(", ");
  //  Serial.print(quat.y(), 6);
  //  Serial.print(", ");
  //  Serial.println(quat.z(), 6);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
