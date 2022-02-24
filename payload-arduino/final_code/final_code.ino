// Name: Jeffrey Matheson
// Language: Arduino
// Project: Rocketry 2021-2022 - Payload
// File: Flight_Code
// Description: The final code for the payload for the FRR launch.
// Date: 2/19/2022
 

// --------------------------------------------------------------------
// Error Code List
//
// - BNO055 #1 and #2 Failure   [2-1-3]
// - BNO055 #1 Failure          [2-1-1]
// - BNO055 #2 Failure          [2-1-2]
// - SD Card Failure            [3-2-1]
// --------------------------------------------------------------------


#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_Rocketry.h>
#include <utility/imumaths.h>
#include <Cardinal.h>


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


// Pin Values
const int pin_Buzzer = 7;                           // Associate the Piezo Buzzer with DIGITAL 7


// BNO055 Objects
Adafruit_BNO055_Rocketry BNO1 = Adafruit_BNO055_Rocketry(55, 0x28);   // BNO055 Object For The 1st BNO055 IMU
Adafruit_BNO055_Rocketry BNO2 = Adafruit_BNO055_Rocketry(55, 0x29);   // BNO055 Object For The 2nd BNO055 IMU


// File objects associated with the BNO055 data
File BNO055_1;                                      // File object for the 1st BNO055 data
File BNO055_2;                                      // File object for the 2nd BNO055 data

// File objects associated with the displacement data
File displacementData;                              // File object for the displacement calculations

// File objects associated with general data
File headingData;                                   // File object for the heading information
File programStartData;                              // File for timestamping the beginning of the program


// Flag Variables
int flag_hasHeading     = 0;    // Flag for marking whether or not the heading of the rocket on the rail has been obtained
int flag_hasLaunched    = 0;    // Flag for marking whether or not the rocket has launched yet
int flag_inFlight       = 0;    // Flag for marking whether or not the rocket is in-flight
int flag_programEnded   = 0;    // Flag for marking whether or not the program was successfully ended


// Variables for data comparison
double heading;                 // Variable for holding the initial compass heading of the rocket


// Variables For Timing
unsigned long dataSampleRate                = 100;                                  // The amount of time (in ms) to delay between BNO055 data samples
unsigned long recordDurationMinutes         = 1;                                    // The amount of time (in minutes) to record BNO055 data
unsigned long recordDurationMilliseconds    = recordDurationMinutes * 60 * 1000;    // The amount of time (in ms) to record BNO055 data
unsigned long startTime                     = millis();                             // The time the program first starts recording BNO055 data; is updated within the for loop
unsigned long currentTimestamp              = startTime;                            // The current time of the program; is updated while BNO055 data is being recorded


// ====================================================================
// Setup & Loop
// ====================================================================

// Program Setup
void setup() {
  // Pin Setup
  pinMode(pin_Buzzer, OUTPUT);          // Set the pin associated with the Piezo Buzzer to OUTPUT

  // BNO055 Initialization
  boolean checkBNO1 = BNO1.begin();     // Attempt to initialize BNO055 #1
  boolean checkBNO2 = BNO2.begin();     // Attempt to initialize BNO055 #2

  if (!checkBNO1 and !checkBNO2) {      // If both BNO055 #1 and #2 fail to initialize, play the associated error code on loop
    while (1) {
      buzzer_playErrorCode(2, 1, 3);    // BNO055 #1 and #2 Failure: 2x Beep - Pause - 1x Beep - Pause - 3x Beep
    }
  } else if (!checkBNO1) {              // If just BNO055 #1 fails to initialize, play the associated error code on loop
    while (1) {
      buzzer_playErrorCode(2, 1, 1);    // BNO055 #1 Failure: 2x Beep - Pause - 1x Beep - Pause - 1x Beep
    }
  } else if (!checkBNO2) {              // If just BNO055 #2 fails to initialize, play the associated error code on loop
    while (1) {
      buzzer_playErrorCode(2, 1, 2);    // BNO055 #2 Failure: 2x Beep - Pause - 1x Beep - Pause - 2x Beep
    }
  }

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

  // BNO055 Calibration
  // ### !!! ### ADD HARDCODED VARIABLES FOR CALIBRATION

  // SD Card Initialization
  if (!SD.begin()) {                    // Try to initialize the SD card (defaults to Pin 10 on the UNO and Pin 53 on the MEGA)
    while (1) {                         // If just the SD Card fails to initialize, play the associated error code
      buzzer_playErrorCode(3, 2, 1);    // SD Card Failure: 3x Beep - Pause - 2x Beep - Pause - 1x Beep
    }
  }

  SD.remove("dataIMU1.csv");    // Remove "dataIMU1.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("dataIMU2.csv");    // Remove "dataIMU2.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("dataDisp.csv");    // Remove "dataDisp.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("dataHead.csv");    // Remove "dataHead.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("strtTime.txt");    // Remove "strtTime.txt" from the SD Card if it already exists; needed for resetting files between runs

  BNO055_1 = SD.open("dataIMU1.csv", FILE_WRITE);   // Open the "dataIMU1.csv" file on the SD Card (in write mode) and associate it with the BNO055_1 file object
  BNO055_1.println("Timestamp,Accelerometer [X],Accelerometer [Y],Accelerometer [Z],Linear Accel. [X],Linear Accel. [Y],Linear Accel. [Z],Gyroscope [X],Gyroscope [Y],Gyroscope [Z],Euler [X],Euler [Y],Euler [Z],Mag [X], Mag [Y], Mag [Z]");    // Print the column headers for BNO055_1
  BNO055_2 = SD.open("dataIMU2.csv", FILE_WRITE);   // Open the "dataIMU2.csv" file on the SD Card (in write mode) and associate it with the BNO055_2 file object
  BNO055_2.println("Timestamp,Accelerometer [X],Accelerometer [Y],Accelerometer [Z],Linear Accel. [X],Linear Accel. [Y],Linear Accel. [Z],Gyroscope [X],Gyroscope [Y],Gyroscope [Z],Euler [X],Euler [Y],Euler [Z],Mag [X], Mag [Y], Mag [Z]");    // Print the column headers for BNO055_2

  displacementData = SD.open("dataDisp.csv", FILE_WRITE);   // Open the "dataDisp.csv" file on the SD Card (in write mode) and associated it with the displacementData file object
  displacementData.println("### !!! ### ADD COLUMN HEADER INFORMATION HERE!!!");    // Print the column headers for displacementData

  headingData = SD.open("dataHead.csv", FILE_WRITE);    // Open the "dataHead.csv" file on the SD Card (in write mode) and associate it with the headingData file object
  headingData.println("Timestamp,Mag [X],Mag [Y],Mag [Z],Yaw,Cardinal String,Cardinal Int");    // Print the column headers for headingData

  programStartData = SD.open("strtTime.txt", FILE_WRITE);   // Open the "strtTime.txt" file on the SD Card (in write mode) and associate it with the programStartData file object
  programStartData.print("Program Start Time (ms): ");    // Print the line starter for programStartData
  programStartData.println(startTime);    // Print the program start time to programStartData
  programStartData.close();   // Close the File object associated with programStartData


  // Indicate Successful Start Up
  buzzer_playStartTone();   // Play the "start up" tone to indicate that the program started successfully
}

// Program Loop
void loop() {
  // Get Cardinal Heading
  //  while (!flag_hasHeading) {
  //    // ### !!! ### Need to ensure that the rocket is on the rail before the heading is read
  //    getCardinalHeading();
  //  }

  // Wait until the rocket has launched
  //  while (!flag_hasLaunched) {
  //    checkForLaunch();
  //  }

  // Record data while the rocket is in-flight
  //  while (flag_inFlight) {
  //    unsigned long currentTimestamp = millis();
  //    writeToSD(BNO055_1, BNO1, currentTimestamp);
  //    writeToSD(BNO055_2, BNO2, currentTimestamp);
  //
  //    calculatedisplacementData();
  //
  //    // Check to see if the rocket has landed and is "at rest"
  //    checkForLand();
  //  }

  startTime = millis();   // Update startTime to right before the BNO055s start recording to ensure they go for the full duration
  currentTimestamp = startTime;   // Update currentTimestamp

  // Record data from the BNO055s for the length of recordDurationMilliseconds
  while ((currentTimestamp - startTime) < recordDurationMilliseconds) {
    currentTimestamp = millis();                  // Update currentTimestamp
    writeToSD(BNO055_1, BNO1, currentTimestamp);  // Save the current data from BNO055 #1 to the SD Card
    writeToSD(BNO055_2, BNO2, currentTimestamp);  // Save the current data from BNO055 #2 to the SD Card
    delay(dataSampleRate);            // Delay for the length of dataSampleRate
  }

  // If the rocket is no longer in flight, end the program
  if (!flag_programEnded) {
    endProgram();
  }

  // Play a double tone on loop to indicate that the program has ended
  while (1) {
    tone(pin_Buzzer, 3500, 100);
    delay(200);
    tone(pin_Buzzer, 4000, 100);
    delay(200);

    delay(500);
  }
}


// ====================================================================
// Helper Functions
// ====================================================================

// --------------------------------------------------------------------
// Piezo Buzzer Related Functions
// --------------------------------------------------------------------

// Function for playing a start-up tone via the Piezo Buzzer
void buzzer_playStartTone() {
  const int duration = 100;             // Variable for holding the length of time for playing the "Chirps"

  // Play 20 loops of bouncing back and forth between a 3 kHz tone and a 4 kHz tone
  for (int i = 0; i < 20; i += 1) {
    tone(pin_Buzzer, 3000, duration);   // Play a 3 kHz tone on the Piezo Buzzer for the length of duration
    delay(duration * 2);                // Delay for the length of duration * 2 to allow the tone to play on the buzzer, and for a pause afterwards
    tone(pin_Buzzer, 4000, duration);   // Play a 4 kHz tone on the Piezo Buzzer for the length of duration
    delay(duration * 2);                // Delay for the length of duration * 2 to allow the tone to play on the buzzer, and for a pause afterwards
  }
}

// Function for playing tones for indicating error codes
void buzzer_playErrorCode(int count_1, int count_2, int count_3) {
  const int durationTone = 250;             // The length of time (in ms) to play a tone
  const int delayCount = 250;               // The length of time (in ms) to pause between the tones making up the first number
  const int delayNext = 750;                // The length of time (in ms) to pause between two numbers
  const int delayEnd = 2000;                // The length of time (in ms) to pause after playing the error code

  for (int c1 = 0; c1 < count_1; c1++) {    // Loop a number of times equal to count_1 and indicate that number
    tone(pin_Buzzer, 4000, durationTone);   // Play a 4 kHz tone on the pin associated with pin_Buzzer for the length of durationTone
    delay(durationTone + delayCount);       // Delay long enough to play the tone, and for a pause between tones
  }

  delay(delayNext - delayCount);            // Delay for a pause between numbers

  for (int c2 = 0; c2 < count_2; c2++) {    // Loop a number of times equal to count_2 and indicate that number
    tone(pin_Buzzer, 4000, durationTone);   // Play a 4 kHz tone on the pin associated with pin_Buzzer for the length of durationTone
    delay(durationTone + delayCount);       // Delay long enough to play the tone, and for a pause between tones
  }

  delay(delayNext - delayCount);            // Delay for a pause between numbers

  for (int c3 = 0; c3 < count_3; c3++) {    // Loop a number of times equal to count_3 and indicate that number
    tone(pin_Buzzer, 4000, durationTone);   // Play a 4 kHz tone on the pin associated with pin_Buzzer for the length of durationTone
    delay(durationTone + delayCount);       // Delay long enough to play the tone, and for a pause between tones
  }

  delay(delayEnd - delayCount);             // Delay for a pause after playing the entire error code
}

// --------------------------------------------------------------------
// SD Card Related Functions
// --------------------------------------------------------------------

// Function for writing information to the SD Card
void writeToSD(File fileName, Adafruit_BNO055 sensorNum, unsigned long timestamp) {
  // Timestamp
  fileName.print(timestamp);    // Print the current timestamp to the specified file
  fileName.print(",");          // Print a separator to the specified file

  double* accelPtr = get_accelerometerData(sensorNum);    // Save the accelerometer data array from the specified BNO055
  double* linAccelPtr = get_linearAccelData(sensorNum);   // Save the linear accelerometer data array from the specified BNO055
  double* gyroPtr = get_angVelocityData(sensorNum);       // Save the gyroscope data array from the specified BNO055
  double* eulerPtr = get_eulerData(sensorNum);            // Save the gyroscope (euler) data array from the specified BNO055
  double* magPtr = get_magnetometerData(sensorNum);       // Save the magnetometer data array from the specified BNO055

  // Accelerometer
  fileName.print(accelPtr[0]);      // Print the x axis accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(accelPtr[1]);      // Print the y axis accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(accelPtr[2]);      // Print the z axis accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file

  // Linear Accelerometer
  fileName.print(linAccelPtr[0]);   // Print the x axis linear accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(linAccelPtr[1]);   // Print the y axis linear accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(linAccelPtr[2]);   // Print the z axis linear accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file

  // Gyroscope
  fileName.print(gyroPtr[0]);       // Print the x axis gyroscope data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(gyroPtr[1]);       // Print the y axis gyroscope data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(gyroPtr[2]);       // Print the z axis gyroscope data to the specified file
  fileName.print(",");              // Print a separator to the specified file

  // Euler
  fileName.print(eulerPtr[0]);      // Print the x axis euler data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(eulerPtr[1]);      // Print the y axis euler data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(eulerPtr[2]);      // Print the z axis euler data to the specified file
  fileName.print(",");              // Print a separator to the specified file

  // Magnetometer
  fileName.print(magPtr[0]);        // Print the x axis magnetometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(magPtr[1]);        // Print the y axis magnetometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.println(magPtr[2]);      // Print the z axis magnetometer data to the specified file
}

// --------------------------------------------------------------------
// Rocket State Related Functions
// --------------------------------------------------------------------

// Function for obtaining the cardinal heading of the rocket once it is on the rail
//void getCardinalHeading() {
//  boolean headingFound = false;
//  double magX, magY;
//  double* magPtr = get_magnetometerData(BNO1);   // Save the gyroscoped (euler) data array from the specified BNO055
//  double calculatedYaw = 0;   // Variable for holding the value of the found heading
//  int cardinal_integer;
//  String cardinal_string;
//  Cardinal cardinal;
//  magX = magPtr[0];
//  magY = magPtr[1];
//
//  // ### !!! ### ADD CALISTA'S CODE HERE
//  calculatedYaw = atan2(magY, magX) * 180 / M_PI;
//  while (calculatedYaw < 0) {
//    calculatedYaw += 360;
//  }
//
//  cardinal_integer = cardinal.getInteger(3, calculatedYaw);
//  cardinal_string = cardinal.getString(3, calculatedYaw);
//
//  if (headingFound) {
//    heading = calculatedYaw;
//    // Magnetometer
//    headingData.print(magPtr[0]);   // Print the x axis euler data to the specified file
//    headingData.print(",");    // Print a separator to the specified file
//    headingData.print(magPtr[1]);   // Print the y axis euler data to the specified file
//    headingData.print(",");    // Print a separator to the specified file
//    headingData.print(magPtr[2]);   // Print the z axis euler data to the specified file
//    headingData.print(",");    // Print a separator to the specified file
//    headingData.print(calculatedYaw);
//    headingData.print(",");    // Print a separator to the specified file
//    headingData.print(cardinal_string);
//    headingData.print(",");    // Print a separator to the specified file
//    headingData.println(cardinal_integer);
//    flag_hasHeading = 1;
//  }
//}

// Function for checking if the rocket has launched
void checkForLaunch() {
  boolean variablesAreDifferent = false;

  // ### !!! ### ADD CODE HERE

  if (variablesAreDifferent) {
    flag_hasLaunched = 1;
    flag_inFlight = 1;
  }
}

// Function for checking if the rocket has landed
void checkForLand() {
  boolean hasLanded = false;

  // ### !!! ### ADD CODE HERE

  if (hasLanded) {
    flag_inFlight = 0;
  }
}

// Function for ending the program (i.e. closing the SD Card files, etc)
void endProgram() {
  BNO055_1.close();           // Close the File object associated with the 1st BNO055
  BNO055_2.close();           // Close the File object associated with the 2nd BNO055

  displacementData.close();   // Close the File object associated with displacementData
  headingData.close();        // Close the File object associated with headingData

  flag_programEnded = 1;      // Set the programEnded flag HIGH so that the function only runs once
}

// --------------------------------------------------------------------
// Displacement Related Functions
// --------------------------------------------------------------------

// Function for calculating the Rocket's displacement
void calculatedisplacementData() {
  // ### !!! ### ADD CODE HERE
}

// --------------------------------------------------------------------
// BNO055 Related Functions
// --------------------------------------------------------------------

// Function for obtaining the accelerometer data from the specified BNO055; returns an array of the three axis values
double* get_accelerometerData(Adafruit_BNO055 sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t accelerometerData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &accelerometerData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->acceleration.x;   // Save the x axis data to xyz
  xyz[1] = event->acceleration.y;   // Save the y axis data to xyz
  xyz[2] = event->acceleration.z;   // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the linear accelerometer data from the specified BNO055; returns an array of the three axis values
double* get_linearAccelData(Adafruit_BNO055 sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t linearAccelData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &linearAccelData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->acceleration.x;   // Save the x axis data to xyz
  xyz[1] = event->acceleration.y;   // Save the y axis data to xyz
  xyz[2] = event->acceleration.z;   // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the gyroscope data from the specified BNO055; returns an array of the three axis values
double* get_angVelocityData(Adafruit_BNO055 sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t angVelocityData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &angVelocityData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->gyro.x;    // Save the x axis data to xyz
  xyz[1] = event->gyro.y;    // Save the y axis data to xyz
  xyz[2] = event->gyro.z;    // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the euler angle data from the specified BNO055; returns an array of the three axis values
double* get_eulerData(Adafruit_BNO055 sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t orientationData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &orientationData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->orientation.x;    // Save the x axis data to xyz
  xyz[1] = event->orientation.y;    // Save the y axis data to xyz
  xyz[2] = event->orientation.z;    // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the magnetometer data from the specified BNO055; returns an array of the three axis values
double* get_magnetometerData(Adafruit_BNO055 sensorNum) {   // Takes in a BNO055 sensor object
  sensors_event_t magnetometerData;   // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &magnetometerData;   // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->magnetic.x;   // Save the x axis data to xyz
  xyz[1] = event->magnetic.y;   // Save the y axis data to xyz
  xyz[2] = event->magnetic.z;   // Save the z axis data to xyz

  return xyz;   // Return the array
}
