// Name: UMass Rocket Team
// Language: Arduino
// Project: Rocketry 2021-2022 - Payload
// Description: The final code for the payload for the Alabama launch (final code v2).
// Date: 4/9/2022


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
#include <EEPROM.h>

#include <active_status.h>
#include <quaternion.h>
#include <fuse_gyro_acc.h>

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
const int pin_Buzzer = 30;                           // Associate the Piezo Buzzer with DIGITAL 30

// EEPROM Address (file naming)
const int EEPROM_ADDRESS = 45;

// BNO055 Objects
Adafruit_BNO055_Rocketry BNO1 = Adafruit_BNO055_Rocketry(55, 0x28);   // BNO055 Object For The 1st BNO055 IMU
Adafruit_BNO055_Rocketry BNO2 = Adafruit_BNO055_Rocketry(55, 0x29);   // BNO055 Object For The 2nd BNO055 IMU

// File objects associated with the BNO055 data
File BNO055_1;                                      // File object for the 1st BNO055 data
File BNO055_2;                                      // File object for the 2nd BNO055 data
File EulerAngles;

// File objects associated with the displacement data
File PeterAccel;                                    // Calculated linear acceleration data
File displacementData;                              // File object for the displacement calculations

// File objects associated with general data
File programStartData;                              // File for timestamping the beginning of the program


// Flag Variables
int flag_hasLaunched    = 0;    // Flag for marking whether or not the rocket has launched yet
int flag_inFlight       = 0;    // Flag for marking whether or not the rocket is in-flight
int flag_programEnded   = 0;    // Flag for marking whether or not the program was successfully ended


// Variables For Timing
unsigned long dataSampleRate                = 50;                                   // The amount of time (in ms) to delay between BNO055 data samples
unsigned long recordDurationMinutes         = 1;                                    // The amount of time (in minutes) to record BNO055 data
unsigned long recordDurationMilliseconds    = recordDurationMinutes * 60 * 1000;    // The amount of time (in ms) to record BNO055 data
unsigned long startTime                     = millis();                             // The time the program first starts recording BNO055 data; is updated within the for loop
unsigned long currentTimestamp              = startTime;                            // The current time of the program; is updated while BNO055 data is being recorded


// Variables For xBee (latitude)
float mapCenterX = 34.895440;                        // center X on gridded map
float mapCenterY = 86.617000;                        // center Y on gridded map


// Setup Complementary (NEW UPDATE)
OrientationFusion gyroacc_Fusion(0, 0, 0); //orientation at (0 roll, 0 pitch, 0 yaw, dt = 0.1)

// ====================================================================
// Setup & Loop
// ====================================================================

// Program Setup
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("ROCKET: POWERED ON"); Serial.println();

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

  // BNO055 Calibration
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);

  Serial.println("OFFSET VALUES");
  displaySensorOffsets(calibrationData);
  Serial.println();

  BNO1.setSensorOffsets(calibrationData);
  foundCalib = true;

  // Find Starting Angles (on Rail)
  Serial.println();
  Serial.println("STARTING ANGLES");

  // SD Card Initialization
  if (!SD.begin()) {                    // Try to initialize the SD card (defaults to Pin 10 on the UNO and Pin 53 on the MEGA)
    while (1) {                         // If just the SD Card fails to initialize, play the associated error code
      buzzer_playErrorCode(3, 2, 1);    // SD Card Failure: 3x Beep - Pause - 2x Beep - Pause - 1x Beep
    }
  }

  SD.remove("dataEule.csv");
  EulerAngles = SD.open("dataEule.csv", FILE_WRITE);

  int i = 0;
  while (i < 10) {
    double* eulerPtr = get_eulerData(BNO1);                 // Save the gyroscope (euler) data array from the specified BNO055
    EulerAngles.print(eulerPtr[0]);                         // Print the x axis gyroscope data to the specified file
    Serial.print(eulerPtr[0]);
    EulerAngles.print(",");                                 // Print a separator to the specified file
    Serial.print(",");
    EulerAngles.print(eulerPtr[1]);                         // Print the y axis gyroscope data to the specified file
    Serial.print(eulerPtr[1]);
    EulerAngles.print(",");                                 // Print a separator to the specified file
    Serial.print(",");
    EulerAngles.println(eulerPtr[2]);                       // Print the z axis gyroscope data to the specified file
    Serial.println(eulerPtr[2]);
    i++;
  }

  //INITAILIZE INITIAL ORIENTATION (NEW UPDATE)
  double* eulerptr = get_eulerData(BNO1);
  gyroacc_Fusion.UpdateInitOrientation(eulerptr[1], eulerptr[0], eulerptr[2]); //Roll: x; pitch: y; z: yaw

  Serial.println();

  // BNO055 Update Settings
  BNO1.setMode(OPERATION_MODE_AMG);     // Set the Operation Mode of the sensor to Accelerometer, Magnetometer, and Gyroscope
  delay(50);
  BNO1.setGRange(0x0F);                 // Set the sensor's G Range to 16Gs
  delay(50);

  BNO2.setMode(OPERATION_MODE_AMG);     // Set the Operation Mode of the sensor to Accelerometer, Magnetometer, and Gyroscope
  delay(50);
  BNO2.setGRange(0x0F);                 // Set the sensor's G Range to 16Gs
  delay(50);

  // SD Card - Handle Files
  SD.remove("dataIMU1.csv");    // Remove "dataIMU1.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("dataIMU2.csv");    // Remove "dataIMU2.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("dataDisp.csv");    // Remove "dataDisp.csv" from the SD Card if it already exists; needed for resetting files between runs
  SD.remove("PeteAccl.csv");

  BNO055_1 = SD.open("dataIMU1.csv", FILE_WRITE);   // Open the "dataIMU1.csv" file on the SD Card (in write mode) and associate it with the BNO055_1 file object
  BNO055_1.println("Timestamp,Accel [X],Accel [Y],Accel [Z],Gyro [X],Gyro [Y],Gyro [Z],Mag [X], Mag [Y], Mag [Z]");    // Print the column headers for BNO055_1
  BNO055_2 = SD.open("dataIMU2.csv", FILE_WRITE);   // Open the "dataIMU2.csv" file on the SD Card (in write mode) and associate it with the BNO055_2 file object
  BNO055_2.println("Timestamp,Accel [X],Accel [Y],Accel [Z],Gyro [X],Gyro [Y],Gyro [Z],Mag [X], Mag [Y], Mag [Z]");    // Print the column headers for BNO055_2

  displacementData = SD.open("dataDisp.csv", FILE_WRITE);   // Open the "dataDisp.csv" file on the SD Card (in write mode) and associated it with the displacementData file object
  displacementData.println("### !!! ### ADD COLUMN HEADER INFORMATION HERE!!!");    // Print the column headers for displacementData

  PeterAccel = SD.open("PeteAccl.csv", FILE_WRITE);
  PeterAccel.println("Timestamp,LinAcc [X],LinAcc [Y],LinAcc [Z]");

  // Indicate Successful Start Up
  buzzer_playStartTone();   // Play the "start up" tone to indicate that the program started successfully

  // Compass Heading
  Serial.println("ROCKET: SEND HEADING ANGLE");
  while (Serial.available() == 0) {
    // wait for GROUND to send data packet
  }
  char received;
  char messageArray[3];
  int j = 0;

  while (j < 3) {
    if (Serial.available() > 0) {
      received = Serial.read();
      messageArray[j] = received;
      j++;
    }
  }

  delay(1000);

  Serial.print("HEADING:  ");
  Serial.println(atoi(messageArray));

  // Wait for xBee confirmation
  Serial.println("ROCKET: PAYLOAD IS READY");
  //Serial.println("ROCKET: READY TO START TIMER?");
}

// Program Loop
void loop() {

  startTime = millis();   // Update startTime to right before the BNO055s start recording to ensure they go for the full duration
  currentTimestamp = startTime;   // Update currentTimestamp

  // Define previous
  double prevLinearAcc[] = {0, 0, 0};
  double prevVel[] = {0, 0, 0};

  // Record data from the BNO055s for the length of recordDurationMilliseconds
  while ((currentTimestamp - startTime) < recordDurationMilliseconds) {
    currentTimestamp = millis();                  // Update currentTimestamp
    //Get new orientation and update it (NEW UPDATE)
    double* accelPtr = get_accelerometerData(BNO1);    // Save the accelerometer data array from the specified BNO055
    double* gyroPtr = get_angVelocityData(BNO1);
    //We can do some KF to get 1 sensor reading here (but later...)
    double accel[] = {accelPtr[0], accelPtr[1], accelPtr[2]};
    double gyro[] = {gyroPtr[0], gyroPtr[1], gyroPtr[2]};
    gyroacc_Fusion.FuseGyroAcc(accel, gyro, 0.1); // 0.1 is dt

    //Orientation to quaternion (NEW UPDATE)
    Quaternion quat;
    quat = quat.euler_to_quaternion(gyroacc_Fusion.roll, gyroacc_Fusion.pitch, gyroacc_Fusion.yaw);
    //Rotate
    auto rotatedGVector = quat.rotate(Quaternion(0, 0, 9.81)); //Rotate vecotr [0,0,9.8] by a quaternion determined by roll, pitch, yaw
    double LinearAcc[] = {accelPtr[0] - rotatedGVector.b, accelPtr[1] - rotatedGVector.c, accelPtr[2] - rotatedGVector.d};

    PeterAccel.print(currentTimestamp);
    PeterAccel.print(",");
    PeterAccel.print(LinearAcc[0]);
    PeterAccel.print(",");
    PeterAccel.print(LinearAcc[1]);
    PeterAccel.print(",");
    PeterAccel.println(LinearAcc[2]);

    // Call calc. displacement
    //calculatedisplacementData(LinearAcc[0]);    

    // Update values
    prevLinearAcc[0] = LinearAcc[0]; 
    prevLinearAcc[1] = LinearAcc[1];
    prevLinearAcc[2] = LinearAcc[2];

      writeToSD(BNO055_1, BNO1, currentTimestamp);  // Save the current data from BNO055 #1 to the SD Card
    writeToSD(BNO055_2, BNO2, currentTimestamp);  // Save the current data from BNO055 #2 to the SD Card
    //Serial.println("ROCKET: RECORDING DATA...");
    delay(dataSampleRate);            // Delay for the length of dataSampleRate
  }

  // If the rocket is no longer in flight, end the program
  if (!flag_programEnded) {
    endProgram();
  }

  // Play a double tone on loop to indicate that the program has ended
  while (1) {
    // Send final coordinate to GROUND
    Serial.println();
    Serial.println("ROCKET: PAYLOAD HAS ENDED");
    Serial.println(getFinalCoord());

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
// EEPROM Related Functions
// --------------------------------------------------------------------

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
  Serial.print("A: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nG: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nM: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");
}

// --------------------------------------------------------------------
// SD Card Related Functions
// --------------------------------------------------------------------

// Function for writing information to the SD Card
void writeToSD(File fileName, Adafruit_BNO055_Rocketry sensorNum, unsigned long timestamp) {
  // Timestamp
  fileName.print(timestamp);    // Print the current timestamp to the specified file
  fileName.print(",");          // Print a separator to the specified file

  double* accelPtr = get_accelerometerData(sensorNum);    // Save the accelerometer data array from the specified BNO055
  double* gyroPtr = get_angVelocityData(sensorNum);       // Save the gyroscope data array from the specified BNO055
  double* magPtr = get_magnetometerData(sensorNum);       // Save the magnetometer data array from the specified BNO055

  // Accelerometer
  fileName.print(accelPtr[0]);      // Print the x axis accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(accelPtr[1]);      // Print the y axis accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(accelPtr[2]);      // Print the z axis accelerometer data to the specified file
  fileName.print(",");              // Print a separator to the specified file


  // Gyroscope
  fileName.print(gyroPtr[0]);       // Print the x axis gyroscope data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(gyroPtr[1]);       // Print the y axis gyroscope data to the specified file
  fileName.print(",");              // Print a separator to the specified file
  fileName.print(gyroPtr[2]);       // Print the z axis gyroscope data to the specified file
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

// Function for ending the program (i.e. closing the SD Card files, etc)
void endProgram() {
  BNO055_1.close();           // Close the File object associated with the 1st BNO055
  BNO055_2.close();           // Close the File object associated with the 2nd BNO055
  EulerAngles.close();
  PeterAccel.close();

  displacementData.close();   // Close the File object associated with displacementData

  flag_programEnded = 1;      // Set the programEnded flag HIGH so that the function only runs once
}

// --------------------------------------------------------------------
// Displacement Related Functions
// --------------------------------------------------------------------

// Function for calculating the Rocket's displacement
void calculatedisplacementData(double dispArr[]) {
  dispArr[0] = 1000; // dummy val, feet
  dispArr[1] = 1000; // dummy val, feet
}

// Function for determining grid coordinate value
String getFinalCoord() {
  // 1. Fetch starting coordinate (center = J10)
  int startCoord[2];
  Serial.println("ROCKET: SEND PACKET NOW");
  getStartingCoord(startCoord);
  int startX = startCoord[0];
  int startY = startCoord[1];

  // 2. Wait for displacement calculations to finish...
  double dispArr[2];
  calculatedisplacementData(dispArr);
  double dispX = dispArr[0];
  double dispY = dispArr[1];

  // 3. Find new grid coordinate based on start
  int endX = round(startX + (dispX / 250));
  int endY = round(startY + (-dispY / 250)); // Y-axis inverted (top = 1) on map

  char letters[21] = {'0', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'};
  String finalCoord = String(letters[endX]) + endY;
  String startString = String(letters[startX]) + startY;

  Serial.print("START COORDINATE: "); Serial.println(startString);
  Serial.print("FINAL COORDINATE: ");

  return finalCoord;
}

// --------------------------------------------------------------------
// xBee Related Functions
// --------------------------------------------------------------------
void getStartingCoord(int startCoord[]) {
  char received = '0';
  char messageArray[25];                               // stores the whole message sent by GROUND
  char ilocX[12];                                      // received x coord values (max value: ###.######)
  char ilocY[12];                                      // received y coord values (max value: ###.######)
  int i_m = 0;                                         // iterating variable used for messageArray
  int i_x = 0;                                         // iterating variable used for ilocX
  int i_y = 0;                                         // iterating variable used for ilocY
  bool is_ground_talking = false;                      // flag to determine if GROUND is currently sending a message
  bool reading_xCoord = true;                          // if true, we are reading x coordinate. if false, we are reading y coordinate.

  float xDistFromCenter;
  float yDistFromCenter;

  while (Serial.available() == 0) {
    // wait for GROUND to send data packet
  }

  while (Serial.available() > 0) {                   // runs if ROCKET receives a message from GROUND; message gets concatinated into x and y coordinates
    is_ground_talking = true;

    received = Serial.read();
    messageArray[i_m] = received;

    if (received == '|') {                           // determines if ROCKET is reading x coord or y coord
      reading_xCoord = false;
      continue;
    }

    if (reading_xCoord) {                            // stores x coords
      if (received != '<') {
        ilocX[i_x] = received;
        i_x ++;
      }
    }
    else {                                           // stores y coords
      if (received != '>') {
        ilocY[i_y] = received;
        i_y ++;
      }
    }

    if (messageArray[i_m] == '>') {                  // runs if GROUND has finished sending message
      is_ground_talking = false;
    }
    i_m ++;
  }

  if ((is_ground_talking == false) && (i_m > 0)) {  // when GROUND has finished sending, message gets turned into floats

    float xVal = atof(ilocX);
    float yVal = atof(ilocY);

    float xlatlonFromCenter = (xVal - mapCenterX) * -1; // needs to be negated becasue left in W is positive
    float ylatlonFromCenter = yVal - mapCenterY;
    long latlonTofeet = (10000 / 90) * 3280.4;

    xDistFromCenter = xlatlonFromCenter * latlonTofeet;
    yDistFromCenter = ylatlonFromCenter * latlonTofeet;

    // ACKNOWLEDGE TRANSMISSION
    Serial.println();
    Serial.print("MESSAGE FROM GROUND RECEIVED");
    Serial.println();

    Serial.print("Initial X Coordinate: ");
    Serial.print(xVal, 6);
    Serial.println();

    Serial.print("Initial Y Coordinate: ");
    Serial.print(yVal, 6);
    Serial.println();
    Serial.println();

    Serial.print("DISPLACEMENT FROM MAP CENTER (Coordinate j10)");
    Serial.println();
    Serial.print("X Direction: ");
    Serial.print(xDistFromCenter);
    Serial.print(" (ft)");
    Serial.println();
    Serial.print("Y Direction: ");
    Serial.print(yDistFromCenter);
    Serial.print(" (ft)");
    Serial.println();
    // SEND ACKNOWLEDGEMENT TO GROUND

    // CLEAN UP
    for (int i = 0 ; i < sizeof(messageArray) ; i++) {
      messageArray[i] = 0;
    }
    for (int j = 0; j < sizeof(ilocX); j++) {
      ilocX[j] = 0;
    }
    for (int k = 0; k < sizeof(ilocY); k++) {
      ilocY[k] = 0;
    }
    i_m = 0;
    i_x = 0;
    i_y = 0;
    reading_xCoord = true;
  }

  startCoord[0] = round(10 + (xDistFromCenter / 250));
  startCoord[1] = round(10 + (-yDistFromCenter / 250)); // Y-axis inverted (top = 1) on map
}


// --------------------------------------------------------------------
// BNO055 Related Functions
// --------------------------------------------------------------------

// Function for obtaining the accelerometer data from the specified BNO055; returns an array of the three axis values
double* get_accelerometerData(Adafruit_BNO055_Rocketry sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t accelerometerData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&accelerometerData, Adafruit_BNO055_Rocketry::VECTOR_ACCELEROMETER);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &accelerometerData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->acceleration.x;   // Save the x axis data to xyz
  xyz[1] = event->acceleration.y;   // Save the y axis data to xyz
  xyz[2] = event->acceleration.z;   // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the linear accelerometer data from the specified BNO055; returns an array of the three axis values
double* get_linearAccelData(Adafruit_BNO055_Rocketry sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t linearAccelData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&linearAccelData, Adafruit_BNO055_Rocketry::VECTOR_LINEARACCEL);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &linearAccelData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->acceleration.x;   // Save the x axis data to xyz
  xyz[1] = event->acceleration.y;   // Save the y axis data to xyz
  xyz[2] = event->acceleration.z;   // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the gyroscope data from the specified BNO055; returns an array of the three axis values
double* get_angVelocityData(Adafruit_BNO055_Rocketry sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t angVelocityData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&angVelocityData, Adafruit_BNO055_Rocketry::VECTOR_GYROSCOPE);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &angVelocityData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->gyro.x;    // Save the x axis data to xyz
  xyz[1] = event->gyro.y;    // Save the y axis data to xyz
  xyz[2] = event->gyro.z;    // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the euler angle data from the specified BNO055; returns an array of the three axis values
double* get_eulerData(Adafruit_BNO055_Rocketry sensorNum) {    // Takes in a BNO055 sensor object
  sensors_event_t orientationData;    // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&orientationData, Adafruit_BNO055_Rocketry::VECTOR_EULER);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &orientationData;    // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->orientation.x;    // Save the x axis data to xyz
  xyz[1] = event->orientation.y;    // Save the y axis data to xyz
  xyz[2] = event->orientation.z;    // Save the z axis data to xyz

  return xyz;   // Return the array
}

// Function for obtaining the magnetometer data from the specified BNO055; returns an array of the three axis values
double* get_magnetometerData(Adafruit_BNO055_Rocketry sensorNum) {   // Takes in a BNO055 sensor object
  sensors_event_t magnetometerData;   // Create a sensors_event_t for holding the data event
  sensorNum.getEvent(&magnetometerData, Adafruit_BNO055_Rocketry::VECTOR_MAGNETOMETER);    // Get the specified data from the specified BNO055 event

  sensors_event_t* event = &magnetometerData;   // Save the data to a sensors_event_t* variable

  static double xyz[3];   // Create an array for holding the three axis values

  xyz[0] = event->magnetic.x;   // Save the x axis data to xyz
  xyz[1] = event->magnetic.y;   // Save the y axis data to xyz
  xyz[2] = event->magnetic.z;   // Save the z axis data to xyz

  return xyz;   // Return the array
}
