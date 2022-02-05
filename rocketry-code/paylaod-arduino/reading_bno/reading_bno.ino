// Name: Calista Greenway
// Language: Arduino
// Project: Rocketry - BNO055 Payload
// Date: 2/5/2022

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 
   Connections
   ===========
   Connect SCL to SCL
   Connect SDA to SDA
   Connect Vin to 3.3V or 5V
   Connect GROUND to common ground
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setup(void)
{
  Serial.begin(115200);
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
//  Serial.print("Current Temperature: ");
//  Serial.print(temp);
//  Serial.println(" C");
//  Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  //Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(", ");
  //Serial.print(" Y: ");
  Serial.print(euler.y());
  //Serial.print(",");
  //Serial.print(" Z: ");
  //Serial.print(euler.z());
  Serial.print("\n");


  delay(BNO055_SAMPLERATE_DELAY_MS);
}
