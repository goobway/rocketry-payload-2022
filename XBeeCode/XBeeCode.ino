
//////////////////////////////////////////////////////////////
// UMass Rocket Team                                        //
// Code for GROUND to send launch pad coordinates to ROCKET //
// by Nicholas Borda                                        //
//////////////////////////////////////////////////////////////


// format for incoming messages from GROUND           <11.1111|22.2222>
// message can be at most 25 characters total including <, >, and |
// Arduino can only process 6 digits of percision (6 digits total, not only after the decimal point)...
// The most likely form will be TWO DIGITS BEFORE THE DECIMAL AND FOUR DIGITS AFTER THE DECIMAL

char received = '0';
char messageArray[25];                               // stores the whole message sent by GROUND
char ilocX[12];                                      // received x coord values (max value: ###.######)
char ilocY[12];                                      // received y coord values (max value: ###.######)
bool reading_xCoord = true;                          // if true, we are reading x coordinate. if false, we are reading y coordinate.
int i_m = 0;                                         // iterating variable used for messageArray
int i_x = 0;                                         // iterating variable used for ilocX
int i_y = 0;                                         // iterating variable used for ilocY
bool is_ground_talking = false;                      // flag to determine if GROUND is currently sending a message

///////////////////////////////////////////////////////////////////////////////
float mapCenterX = 34.895440;                        // center X on gridded map 
float mapCenterY = 86.617000;                        // center Y on gridded map 
///////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600); 
}

void loop() {
    //Serial.print("rocket talking!");
    //Serial.println();

  if (Serial.available() > 0) {                      // runs if ROCKET receives a message from GROUND; message gets concatinated into x and y coordinates
    //Serial.print("READING");
    //Serial.println();
    is_ground_talking = true;
    
    received = Serial.read();
    messageArray[i_m] = received;

    if (received == '|'){                            // determines if ROCKET is reading x coord or y coord
      reading_xCoord = false;
      return;
    }
    
    if (reading_xCoord == true){                     // stores x coords
      if (received != '<'){
        ilocX[i_x] = received;
        i_x ++;
      }
    }
    else{                                            // stores y coords
      if (received != '>'){
        ilocY[i_y] = received;
        i_y ++;
      }
    }
    
    if (messageArray[i_m] == '>') {                  // runs if GROUND has finished sending message
      is_ground_talking = false;
      //Serial.print("FINSHED");
      //Serial.println();
    }
    i_m ++;
  }

  
  if ((is_ground_talking == false)&&(i_m > 0)){      // when GROUND has finished sending, message gets turned into floats
    //Serial.print("THINKING");
    //Serial.println();

    float xVal = atof(ilocX);
    float yVal = atof(ilocY);

    float xlatlonFromCenter = (xVal - mapCenterX) * -1; // needs to be negated becasue left in W is positive
    float ylatlonFromCenter = yVal - mapCenterY; 
    long latlonTofeet = (10000/90)*3280.4;

    float xDistFromCenter = xlatlonFromCenter * latlonTofeet;
    float yDistFromCenter = ylatlonFromCenter * latlonTofeet;
    
    
    /////////////////////////////////  ACKNOWLEDGE TRANSMISSION  //////////////////////////////////
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
    //////////////////////////////  SEND ACKNOWLEDGEMENT TO GROUND  ///////////////////////////////
    
    
    
    ///////////////////////////////  CLEAN UP, CLEAN UP, EVERYBODY EVERYWHERE  //////////////////////////////
    for (int i=0 ; i<sizeof(messageArray) ; i++){
      messageArray[i] = 0;
    }
    for (int j=0;j<sizeof(ilocX);j++){
      ilocX[j] = 0;  
    }
    for (int k=0;k<sizeof(ilocY);k++){
      ilocY[k] = 0;
    }
    i_m = 0;
    i_x = 0;
    i_y = 0;
    reading_xCoord = true;
    //////////////////////////////  CLEAN UP, CLEAN UP, EVERYBODY DO YOUR SHARE  /////////////////////////////////
  }
}
