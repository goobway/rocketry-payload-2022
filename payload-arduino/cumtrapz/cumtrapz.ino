/*
   Name: Calista Greenway
   Language: Arduino
   Project: Rocketry - BNO055 Payload
   Date: 2/19/2022
*/

#include <SD.h>
#include <CSV_Parser.h>

// SD vars
File bno1;

void setup() {
  /* initialize */
  Serial.begin(115200);
  delay(1000);
}

void loop(void) {
  // Find total number of populated rows in CSV
  double accX[100];
  double accY[100];
  double accZ[100];  


  delay(100);
}
/*
      xVel = zeros(length(xAcc),1);
      xDis = zeros(length(xAcc),1);
      
      for i = 1:length(timeStep)-1
        b = xAcc(i+1);
        a = xAcc(i);
        xVel(i) = (b+a)/2*tstep;
      end
      
      xVelCur = zeros(length(xVel),1);
      for i = 2:length(xVelCur)
        xVelCur(i) = xVelCur(i-1) + xVel(i);
      end
      
      for i = 1:length(timeStep)-1
        b = xVelCur(i+1);
        a = xVelCur(i);
        xDis(i) = (b+a)/2*tstep;
      end
      
      xDisTot = sum(xDis)
      
      xDisCur = zeros(length(xDis),1);
      for i = 2:length(xDisCur)
        xDisCur(i) = xDisCur(i-1) + xDis(i);
      end
*/
