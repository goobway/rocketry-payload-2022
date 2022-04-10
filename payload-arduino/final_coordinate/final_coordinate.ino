/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Payload
   Desription: Conversion of final displacement to grid coordinate
   Date: 3/23/2022
*/

double offsetX = 250;       // value from xBee, feet
double offsetY = 250;       // value from xBee, feet
int startX = 10;            // feet
int startY = 10;            // feet
double dispX = 500;         // meters
double dispY = 500;         // meters
int endX;
int endY;
String startCoord = "J10";
String finalCoord;

char letters[21] = {'0', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'};

void setup() {
  Serial.begin(115200);
  delay(1000);
  getFinalCoord();
}

void loop() {
  // none
}

void getFinalCoord(){
  // 1. Fetch starting coordinate (center = J10)
  Serial.print("Starting grid coordinate: ");
  getStartCoord();
  Serial.println(startCoord);
  Serial.println("");
  
  // 2. Wait for displacement calculations to finish...
  Serial.println("Calculating displacement...");
  // findDisplacement();
  Serial.print("X (m): ");
  Serial.println(dispX);
  Serial.print("Y (m): ");
  Serial.println(dispY);
  Serial.println("");
  Serial.println("");
  delay(1000);
  
  // 3. Convert final displacement from meters to feet
  Serial.println("Converting to feet...");
  dispX = dispX * 3.281;
  dispY = dispY * 3.281;
  
  Serial.print("X (ft): ");
  Serial.println(dispX);
  Serial.print("Y (ft): ");
  Serial.println(dispY);
  Serial.println("");
    
  // 4. Find new grid coordinate based on start
  endX = round(startX + (dispX / 250));
  endY = round(startY + (dispY / 250));
  finalCoord = String(letters[endX]) + endY;
  Serial.print("Final coordinate: ");
  Serial.print(finalCoord);
  Serial.println("");

  return finalCoord;
}

void getStartCoord() {
  offsetX = offsetX / 250;
  offsetY = offsetY / 250;
  startX = round(startX + offsetX);
  startY = round(startY + offsetY);
  startCoord = String(letters[startX]) + startY;
  
  return startCoord;
}
