// SD CARD INFORMATION HANDLER

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_Rocketry.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// Operation Mode
#define OPERATION_MODE_AMG            0x07


// EEPROM Address
int EEPROM_ADDRESS = 45;


// BNO055 Objects
Adafruit_BNO055_Rocketry BNO1 = Adafruit_BNO055_Rocketry(55, 0x28);   // BNO055 Object For The 1st BNO055 IMU
Adafruit_BNO055_Rocketry BNO2 = Adafruit_BNO055_Rocketry(55, 0x29);   // BNO055 Object For The 2nd BNO055 IMU

// File objects associated with the BNO055 data
File BNO055_1;                                      // File object for the 1st BNO055 data
File BNO055_2;                                      // File object for the 2nd BNO055 data

void writeIntIntoEEPROM(int address, int number)
{ 
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

int readIntFromEEPROM(int address)
{
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

void setup() {
  Serial.begin(9600);
  delay(100);

  // BNO055 Update Settings
  BNO1.setMode(OPERATION_MODE_AMG);     // Set the Operation Mode of the sensor to Accelerometer, Magnetometer, and Gyroscope
  delay(50);
  BNO1.setGRange(0x0F);                 // Set the sensor's G Range to 16Gs
  delay(50);

  BNO2.setMode(OPERATION_MODE_AMG);     // Set the Operation Mode of the sensor to Accelerometer, Magnetometer, and Gyroscope
  delay(50);
  BNO2.setGRange(0x0F);                 // Set the sensor's G Range to 16Gs
  delay(50);

  // SD Card Initialization
  if (!SD.begin()) {                    // Try to initialize the SD card (defaults to Pin 10 on the UNO and Pin 53 on the MEGA)
    while (1) {                         // If just the SD Card fails to initialize, play the associated error code
      //buzzer_playErrorCode(3, 2, 1);    // SD Card Failure: 3x Beep - Pause - 2x Beep - Pause - 1x Beep
    }
  }

  // Check Value Stored in EEPROM
  int fileNumber = readIntFromEEPROM(EEPROM_ADDRESS);
  
  Serial.println(""); Serial.print("Number found at startup: "); Serial.println(fileNumber);
  Serial.println(""); Serial.println("Creating new file name...");

  /***** ALTERNATE METHOD ******/
  //String temp_filename1 = "dataIMU1_" + String(fileNumber) + ".csv";
  //String temp_filename2 = "dataIMU2_" + String(fileNumber) + ".csv";
  //char filename1[15]; temp_filename1.toCharArray(filename1, 15);
  //char filename2[15]; temp_filename2.toCharArray(filename2, 15);
  /****************************/

  char filename1[12]; sprintf(filename1, "dataB1_%d.csv", fileNumber);
  char filename2[12]; sprintf(filename2, "dataB2_%d.csv", fileNumber);
 
  // Increment EEPROM Number (max = 9)
  if (fileNumber != 9){
    fileNumber += 1;
  } else {
    fileNumber = 1;
  }
  writeIntIntoEEPROM(EEPROM_ADDRESS, fileNumber);

  SD.remove(filename1);    // Remove "dataIMU1_#.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove(filename2);    // Remove "dataIMU2_#.csv" from the SD Card if it already exists; needed for resetting files between runs

  BNO055_1 = SD.open(filename1, FILE_WRITE);   // Open the "dataIMU1_#.csv" file on the SD Card (in write mode) and associate it with the BNO055_1 file object
  BNO055_1.println("Timestamp,Accel [X],Accel [Y],Accel [Z],Gyro [X],Gyro [Y],Gyro [Z],Mag [X], Mag [Y], Mag [Z]");    // Print the column headers for BNO055_1
  BNO055_2 = SD.open(filename2, FILE_WRITE);   // Open the "dataIMU2_#.csv" file on the SD Card (in write mode) and associate it with the BNO055_2 file object
  BNO055_2.println("Timestamp,Accel [X],Accel [Y],Accel [Z],Gyro [X],Gyro [Y],Gyro [Z],Mag [X], Mag [Y], Mag [Z]");    // Print the column headers for BNO055_2

}

void loop() {
  BNO055_1.close();           // Close the File object associated with the 1st BNO055
  BNO055_2.close();           // Close the File object associated with the 2nd BNO055
}
