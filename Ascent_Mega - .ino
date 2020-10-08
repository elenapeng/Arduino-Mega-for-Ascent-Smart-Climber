/*  Ascent Motor setup testing used to develop arduino motor control
 * 
 *                Pulse Sensor
 *                    [ ]
 *    Truck 1 Home    | |   Truck 2 Home
 *                    | |
 *           ====---- | | ----====
 *                    | |
 *           Truck 1  | |   Truck 2
 *           ____     | |     ____
 *          |====|___ | | ___|====|
 *                    | |       
 *              ^CW   | |    CCW^              
 *          __________|_|__________
 *                    [ ]
 *           Middle Stroke Sensor
 *                    
 */
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define CW HIGH               //define Clockwise direction as HIGH output
#define CCW LOW               //define Counter Clockwise direction as LOW output
#define ENABLE HIGH           //define Enable value for motor enable input
#define DISABLE LOW           //define Disable value for motor enable input
#define LONG_PUSH_TIME 550    //define minimum time (ms) for long push on remote control
#define DOUBLE_CLICK_TIME 150 //define time (ms) to check for double clicking on remote control

const int RemoteControl = 2;    //define pinout for remote control input
const int BeltDirOut = 12;      //define pinout for belt direction motor relay
const int BeltPowerOut = 11;    //define pinout for belt direction power relay  
const int TiltDirOut = 10;      //define pinout for tilt direction relay
const int TiltSelectOut = 9;    //define pinout for tilt select relay
const int TiltMax = 75;   
const int BluetoothInit = 300;  //ms delay time for Bluetooth to re-init
const float ratioCTA = 0.30;    //ratio for tilt angle measurement filter
const float ratioOA = 0.70;     //ratio for tilt angle measurement filter
bool stringComplete = false;    //USB string complete flag
bool stringCompleteBT = false;  //Bluetooth string complete flag
bool stringCompleteUNO = false; //UNO string complete flag
bool holdBTcom = false;       //stop transmitting on Serial2 Bluetooth com when true
String inputString = "";       //a String to hold incoming data from my apple computer
String inputStringBT = "";     //a String to hold incoming data from bluetooth
String inputStringUNO = "";    //a String to hold incoming data from UNO
unsigned int tilt = 0;         //set tilting angle
float targetAngle;             //angle intend to move to
unsigned long int accUnoTime = 0;   //accumulated time
unsigned long int accUnoDist = 0;   //accumulated distance

Adafruit_MMA8451 mma = Adafruit_MMA8451();

void setup() {
  Serial.begin(9600);            //Initialize serial data from Computer USB to Mega
  Serial1.begin(9600);           //Initialize serial data from Mega to Uno 
  Serial2.begin(9600);           //Initialize serial data from Bluetooth to Mega                          
  inputString.reserve(20);       // reserve 20 bytes for the inputString:
  pinMode(RemoteControl,INPUT_PULLUP);
  pinMode(BeltDirOut, OUTPUT);
  pinMode(BeltPowerOut, OUTPUT);
  pinMode(TiltDirOut, OUTPUT);
  pinMode(TiltSelectOut, OUTPUT);
  digitalWrite(BeltDirOut, LOW);          
  digitalWrite(BeltPowerOut, LOW);          
  digitalWrite(TiltDirOut, LOW); 
  digitalWrite(TiltSelectOut, LOW);
  Serial.println("Adafruit MMA8451 test");  //Start the accelerometer to measure tilt angle
  if (! mma.begin()) {                      //mma.beging() returns true if mm8451 responds correctly
    Serial.println("Couldnt start");
  }
  Serial.println("MMA8451 found");
  mma.setRange(MMA8451_RANGE_2_G);          //set up using 2g range
  Serial.print("Range = "); 
  Serial.print(2 << mma.getRange());        //make sure range was set
  Serial.println("G");
}

void loop() {  
  if (stringComplete) {                   //loop until stringComplete from Computer USB
    processComputerCommand(inputString);  //process the command 
    inputString = "";                     //clear the string:
    stringComplete = false;               //end of if(stringComplete)
  } 
  if (stringCompleteBT) {                   //loop until stringComplete from serialEvent Bluetooth
    processBTCommand(inputStringBT);
    inputStringBT = "";                    //clear the string:
    stringCompleteBT = false;              //end of if(stringComplete)
  } 
  if (stringCompleteUNO) {                 //loop until stringComplete from serialEvent Arduino UNO
    processUNOCommand(inputStringUNO);
    inputStringUNO = "";                    //clear the string:
    stringCompleteUNO = false;              //end of if(stringComplete)
  } 
  remotePushCheck();
}                                        //end of loop()

/*----------------------------------------------------------------------------
 * void serialEvent()
 * A serial event occurs whenever new data arrives at the hardware serial port.
 * Once the newline character is received, the string complete flag (stringComplete) is set.
 * The main loop polls this flag every iteration to see if a new complete string is ready.
 * serialEvent() primary purpose is to monitor the USB port on the Arduino Mega
*/
void serialEvent() {                          
  while (Serial.available()) {
    char inChar = (char)Serial.read();           // get the new byte:
    inputString += inChar;                       // add it to the inputString:
    if (inChar == '\n') {                        // if the incoming character is a newline, set a flag so the main loop can
      stringComplete = true;                     // do something about it:
    }
  }
}

/*----------------------------------------------------------------------------
 * void serialEvent1()
 * A serial event occurs whenever new data arrives at the hardware serial port connected to the UNO module.
 * Once the newline character is received, the string complete flag (stringCompleteUNO) is set.
 * The main loop polls this flag every iteration to see if a new complete string is ready.
 * serialEvent1() primary purpose is to monitor the serial port connected to the Arduino UNO motor controller.
*/
void serialEvent1() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();           // get the new byte:
    inputStringUNO += inChar;                       // add it to the inputString:
    if (inChar == '\n') {                        // if the incoming character is a newline, set a flag so the main loop can
      stringCompleteUNO = true;                  // do something about it:
    }
  }
}

/*----------------------------------------------------------------------------
 * void serialEvent2()
 * A serial event occurs whenever new data arrives at the hardware serial port connected to the bluetooth module.
 * Once the newline character is received, the string complete flag (stringCompleteBT) is set.
 * The main loop polls this flag every iteration to see if a new complete string is ready.
 * serialEvent1() primary purpose is to monitor the serial port connected to the Bluetooth adapter linked to the tablet.
*/
void serialEvent2() {
  while (Serial2.available()) {
    char inChar = (char)Serial2.read();           // get the new byte:
    inputStringBT += inChar;                       // add it to the inputString:                       
    if (inChar == '\n') {                        // if the incoming character is a newline, set a flag so the main loop can
       stringCompleteBT = true;                   // do something about it:
    }
  }
}

/*--------------------------------------------------------------------------------------------------------------------------
 * float currentTilt()
 * Reads the tilt sensor and returns this value.
 */
float currentTilt() {
  float angle; 
  mma.read();
  sensors_event_t event;                                //Get a new sensor event 
  mma.getEvent(&event);
  angle =atan2(-mma.y,-mma.z)*57.2957795+180;
  angle = angle -180;                                   //No filter was used because only meause when not moving (no vibration)
  return(angle);
  }

/*-------------------------------------------------------------------------------------------------------------------------
 * void adjustTilt(float targetAngle)
 * Reads the current angle and then moves to a new target angle.  ratioCTA and ratioOA are used to filter the angle 
 * measurement.
 */
void adjustTilt(float targetAngle) {
   float currentTiltAngle;
   float filterAngle;
   float oldAngle;
//   const float ratioCTA = 0.30;
//   const float ratioOA = 0.70;
   long int motionTime = 3000;
   long int startMotionTime = millis();
   long int tiltTime = 0;
   currentTiltAngle = currentTilt();
   if(currentTiltAngle < 53 || currentTiltAngle > 78) {       //reset tilt sensor
    mma.setRange(MMA8451_RANGE_2_G);
    currentTiltAngle = currentTilt();
   }
   oldAngle = oldAngleAve();
   //runAveTilt(true,oldAngle);
   filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
    if (filterAngle < targetAngle) {
      digitalWrite(TiltDirOut, LOW);          //set tilt direction
      digitalWrite(TiltSelectOut, HIGH);      //turn on actuator
      motionTime = (targetAngle - filterAngle) * 1400;
      constrain(motionTime,3000,28000);
      while((filterAngle < targetAngle) && (tiltTime < motionTime))  {
        currentTiltAngle = currentTilt();
        //Serial.print(currentTiltAngle);
        //Serial.print(" ");
        //Serial.print(ratioCTA * currentTiltAngle);
        //Serial.print(" ");
        //Serial.print(ratioOA * oldAngle);
        //Serial.print(" ");
        filterAngle = (ratioCTA * currentTiltAngle) + (ratioOA * oldAngle);
        //Serial.print(runAveTilt(false,filterAngle));
        //Serial.print(" ");
        Serial.print(" Current angle: ");
        Serial.print(filterAngle);
        Serial.print(" ");
        //Serial.println(tiltTime);
        Serial.println("degrees.");
        oldAngle = filterAngle;
        tiltTime = millis() - startMotionTime;
      }  
    }
    else { 
      digitalWrite(TiltDirOut, HIGH);       //set tilt direction
      digitalWrite(TiltSelectOut, HIGH);    //turn on actuator
      motionTime = (filterAngle -targetAngle) * 1400;
      constrain(motionTime,3000,28000);
      while((filterAngle > targetAngle) && (tiltTime < motionTime)){
        currentTiltAngle = currentTilt();
        //Serial.print(currentTiltAngle);
        //Serial.print(" ");
        //Serial.print(ratioCTA * currentTiltAngle);
        //Serial.print(" ");
        //Serial.print(ratioOA * oldAngle);
        //Serial.print(" ");
        filterAngle = (ratioCTA * currentTiltAngle) + (ratioOA * oldAngle);
        //Serial.print(runAveTilt(false,filterAngle));
        //Serial.print(" ");
        Serial.print(" Moving to target angle, now at: ");
        Serial.print(filterAngle);
        Serial.print(" ");
        //Serial.println(tiltTime);
        Serial.println(" degrees.");
        oldAngle = filterAngle;
        tiltTime = millis() - startMotionTime;
      }
    }
    digitalWrite(TiltSelectOut, LOW);
    Serial.print("Now you're at your target angle: ");
    Serial.print(filterAngle);
    Serial.println("degrees.");
  }

/*---------------------------------------------------------------------------------------
 * float runAveTilt(bool initialize, float newTilt)
 * Can initialize and store a running average of arySize numbers.  Developed but not used
 * for the climber tilt function void adjustTilt(float targetAngle).
 */
float runAveTilt(bool initialize, float newTilt) {
  const int arySize = 6;
  static float runAveArray[arySize];
  float runAveReturn = 0;
  int i = 0;
  if(initialize) {
    for(i=0;i<arySize;i++) {
      runAveArray[i] = newTilt;
    }
    runAveReturn = newTilt * arySize;
  }
  else {
    for(i=0;i<arySize-1;i++) {
      runAveArray[i] = runAveArray[i+1];    
    }
    runAveArray[arySize-1] = newTilt;
    for(i=0;i<arySize;i++) {
      runAveReturn += runAveArray[i];
    }
  }
  return(runAveReturn/arySize);
}

/*-----------------------------------------------------------------------------------------------------
 * void homeTilt()
 * 
 */
 void homeTilt() {
    float currentTiltAngle;
    float filterAngle;
    float oldAngle;
    const float ratioCTA = 0.05;
    const float ratioOA = 0.95;
    currentTiltAngle = currentTilt();
    oldAngle = oldAngleAve();
    filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
      digitalWrite(TiltDirOut, LOW);
      digitalWrite(TiltSelectOut, HIGH);
      Serial.print("Starting angle: ");
      Serial.print(filterAngle);
      Serial.println("degrees.");
    while(filterAngle < TiltMax) {
      currentTiltAngle = currentTilt();
      filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
      Serial.print("Moveing to home angle, now at: ");
      Serial.print(filterAngle);
      Serial.println("degrees.");
      oldAngle = filterAngle; 
    }
    digitalWrite(TiltSelectOut, LOW);
  }

/*-----------------------------------------------------------------------------------------------------
 * float oldAngleAve()
 * 
 */
 float oldAngleAve() {
    float oldAngle;
    oldAngle = 0;
    for (int i = 0; i <= 6; i++) {
      oldAngle +=  currentTilt();
    }   
    return(oldAngle/7);
 }

/*---------------------------------------------------------------------------------------------------
 * void findMaxAngle(int increase)
 * 
 */
void findMaxAngle(int increase) {
    float currentTiltAngle;
    float filterAngle;
    float oldAngle;
    const float ratioCTA = 0.05;
    const float ratioOA = 0.95;
    int counter = 0;
    currentTiltAngle = currentTilt();
    oldAngle = oldAngleAve();
    filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
      digitalWrite(TiltDirOut, LOW);
      digitalWrite(TiltSelectOut, HIGH);
      Serial.print("Starting angle: ");
      Serial.print(filterAngle);
      Serial.println("degrees.");
    while(filterAngle < (TiltMax+increase) && counter < 10) {
      currentTiltAngle = currentTilt();
      filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
      Serial.print("Moveing to home angle, now at: ");
      Serial.print(filterAngle);
      Serial.println("degrees.");
      oldAngle = filterAngle;
      counter++; 
    }
    digitalWrite(TiltSelectOut, LOW);
}

/*--------------------------------------------------------------------------------------------------------
 * void findMinAngle(int decrease)
 * 
 */
void findMinAngle(int decrease) {
    float currentTiltAngle;
    float filterAngle;
    float oldAngle;
    const float ratioCTA = 0.05;
    const float ratioOA = 0.95;
    int counter = 0;
    const float TiltMin = 50;
    currentTiltAngle = currentTilt();
    oldAngle = oldAngleAve();
    filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
      digitalWrite(TiltDirOut, HIGH);
      digitalWrite(TiltSelectOut, HIGH);
      Serial.print("Starting angle: ");
      Serial.print(filterAngle);
      Serial.println("degrees.");
    while(filterAngle > (TiltMin-decrease) && counter < 10) {
      currentTiltAngle = currentTilt();
      filterAngle = ratioCTA*currentTiltAngle + ratioOA*oldAngle;
      Serial.print("Moveing to home angle, now at: ");
      Serial.print(filterAngle);
      Serial.println("degrees.");
      oldAngle = filterAngle;
      counter++; 
    }
    digitalWrite(TiltSelectOut, LOW);
}

/*-----------------------------------------------------------------------------------------------------------
 * void beltTension(bool beltDirection)
 * Moves belt linear actuator to either tighten or loosen the drive belt allowing assisted climbing or no 
 * assist climbing.  beltDirection low will tighten the belt and beltDirection high will loosen the belt.
 */
void beltTension(bool beltDirection) {
   digitalWrite(TiltSelectOut, LOW);
   digitalWrite(BeltDirOut, beltDirection);
   digitalWrite(BeltPowerOut, HIGH);
   delay(600); 
   digitalWrite(BeltPowerOut, LOW);
}

/*----------------------------------------------------------------------------------------------------------
 * void processComputerCommand(String cString)
 * Called by loop() function will process the string letter and number read in at the serial port.  The letter and
 * number are read by the serial port event function in loop().  Here they are processed to execute various
 * commands.
 */
void processComputerCommand(String cString) {
  char cType = cString[0];         //first character in string is letter command type
  int cInt = cString[1] - 48;     //convert ASCII char to integer
  cInt = constrain(cInt,0,9);
  switch (cType) {  
  case 'M':                           //report "M" (Motor speed) back to serial port
    Serial.print(cString); 
    Serial1.print(cString);     
  break;
  case 'H':                           //report "H" (Home) back to serial port
    Serial.print(cString); 
    Serial1.print(cString);     
  break;
  case 'S':                           //report "S" (Stroke) back to serial port
     Serial.print(cString); 
     Serial1.print(cString);     
  break;
  case 'T':                           //read current angle without filter 
     //tilt = cInt;
     Serial.print("Current tilt: ");
     Serial.print(currentTilt());
     Serial.print(" degrees");
     Serial.println();
   break;
   case 'A':
     Serial.print("Moving to angle:");   //go to specific angle (int) 
     targetAngle=55+cInt*2.2;            //calculate tilt angle: A0 = 55, A9 = 74.8
     Serial.println(targetAngle);
     adjustTilt(targetAngle);
   break;
   case 'G':                           //go to home tilt angle 
     homeTilt();
   break;
   case 'F':
     findMinAngle(cInt);         //find the maximum angle (findMaxAngle) or to find the minimum angle (findMinAngle)
   break;
   case 'B':                           //change belt tension
     beltTension(cInt);          //0 = tighten, 1= loosen
   break;
   case 'C':
     if(cInt == 0) {                //computer requested Bluetooth communication halted
       holdBTcom = true; 
       accUnoTime = 0;
       accUnoDist = 0;   
     }
     if(cInt == 1) {                //computer requested Bluetooth communication resume
       holdBTcom = false;
       Serial.print(accUnoTime);
       Serial.print(" ");
       Serial.println(accUnoDist);
       Serial2.print(accUnoTime);
       Serial2.print(" ");
       Serial2.println(accUnoDist);
     }
   break;
   case 'D':                           //Demo command: Tilt to target angle, Motor starts, Wait, Motor stop, Home truck, home tilt
     Serial.print("Moving to angle:");   //go to specific angle (int) 
     targetAngle=55+cInt*2.2;  
     Serial.println(targetAngle);
     adjustTilt(targetAngle);
     Serial1.println("S4");
     Serial.print("Motor starts!");
     Serial1.println("M1");
     delay(30000);
     Serial.print("Motor Stops!");
     Serial1.println("M0");
     homeTilt();
     Serial.print("Step off!"); 
   break;
   default:                                                                              //if nothing else matches, do the default
     Serial.println("Command not recognized! Please input M0-9, S0-9, H0, A0, G0, or F0-9.");           //E0 error 0 command not recognized
   break;
   }
}

/*----------------------------------------------------------------------------------------------------------
 * void processBTCommand(String btString)
 * Called by loop() function will process the string letter and number read in at the serial port connected to
 * the Bluetooth module.  The letter and number are read by the serial port event function in loop().  Here they are processed to execute various
 * commands.
 */
void processBTCommand(String btString) {
  char btType = btString[0];         //first character in string is letter command type
  int btInt = btString[1] - 48;     //convert ASCII char to integer 
  btInt = constrain(btInt,0,9);
  Serial.print("BT:");
  Serial.print(btString);
  switch (btType) {  
    case 'H':                           //request motor control home
      Serial1.print(btString);
    break;
    case 'S':                           //set stroke distance to motor control
      Serial1.print(btString); 
    break;
    case 'M':                           //set motor speed to motor control
      Serial1.print(btString);
    break;
    case 'C':
      if(btInt == 0) {                //Tablet requested communication halted
        holdBTcom = true; 
        accUnoTime = 0;
        accUnoDist = 0;   
      }
      if(btInt == 1) {                //Tablet requested communication resume
        String printString;
        String printdist;
        printString = String(accUnoTime);
        printString.concat(' ');
        printdist = String(accUnoDist);
        printString.concat(printdist);
        holdBTcom = false;
        delay(BluetoothInit);         //delay time for tablet to reinit Bluetooth
        //Serial.print(accUnoTime);
        //Serial.print(' ');
        //Serial.println(accUnoDist);
        Serial.println(printString);
        //Serial2.print(accUnoTime);
        //Serial2.print(' ');
        //Serial2.println(accUnoDist);
        Serial2.println(printString);
        Serial2.flush();
        accUnoTime = 0;               //Tablet sends C1 on start up
        accUnoDist = 0;
      }
    break;
    case 'T':                           //read current angle without filter 
      //tilt = btInt;
      Serial.print("Current tilt BT: ");
      Serial.print(currentTilt());
      Serial.print(" degrees");
      Serial.println();
    break;
    case 'A':
     Serial.print("BT: Moving to angle:");   //go to specific angle (int) 
     targetAngle=55+btInt*2.2;            //calculate tilt angle: A0 = 55, A9 = 74.8
     Serial.println(targetAngle);
     adjustTilt(targetAngle);
    break;
    case 'G':                           //go to home tilt angle 
      homeTilt();
    break;
    case 'F':
      //findMaxAngle(commandInt);         //find the maximum angle (findMaxAngle) or to find the minimum angle (findMinAngle)
    break;
    case 'B':                           //change belt tension
      beltTension(btInt);          //0 = tighten, 1= loosen
    break;
    default:                                                                              //if nothing else matches, do the default
      Serial.println("Command Error BT! Please input M0-9, S0-9, H0, A0, G0, or F0-9.");           //E0 error 0 command not recognized
    break;
  } 
}

/*--------------------------------------------------------------------------------------------------------------------------------------
 * void processUNOCommand(String unoString)
 * Called by loop() function will process the string letter and number read in at the serial port connected to the motor control 
 * Arduino UNO.  These strings are generally passed through the Arduino Mega to either the USB serial port or the tablet connected to
 * the Bluetooth module.
 */
void processUNOCommand(String unoString) {
    char commandTypeUNO = ' ';
    commandTypeUNO = (char)unoString[0];         //first character in string is letter command type
    switch (commandTypeUNO) {  
      case '=':                           //report back to serial port
        Serial.print(unoString);          //Echo input string back to serial port
      break;
      case 'T':                           //Check T for time
        unoString[0] = ' ';               //replace T with space
        if(unoString[1] == 'D') {         //check D for distance
          unoString[1] = ' ';             //replace D with space
          unoString.trim();               //trim out the spaces
          if(holdBTcom == false) {
            Serial.println(unoString);          //Echo time and distance back to serial port
            Serial2.println(unoString);         //Send string to Bluetooth
          }
          else {                                //accumulate time and distance instead of sending to Bluetooth com
            accumulateTimeDist(unoString);
         }
      }
      break;
      default:                            //if nothing else matches, do the default
        Serial.print(unoString);          //Echo input string back to serial port
      break;
   }  
}

/*-------------------------------------------------------------------------------------------------------------------
 * void accumulateTimeDist(String accString)
 * Accumulates time and distance when the tablet connected via Bluetooth requests no update.  The tablet would request
 * no update while the tablet app is on another page.  The tablet drops the Bluetooth connection when it switches pages,
 * so it will request no updating during this time.  The accumulate function will save the time and distance into accUnoTime
 * and accUnoDist variables to send to the tablet when the tablet returns to the performance page and allows updates to 
 * resume.
 */
void accumulateTimeDist(String accString) {
  char unoTime[10];
  char unoDist[10];
  String bufferString;
  int stringCnt = 0;
  int stringCntOffset = 0;
  int unoStringLength = accString.length();
  bool unoTimeBool = true;                      //set to read out time from unoString
  char unoChar = ' ';
  while((stringCnt < unoStringLength)&&(stringCnt < 10)) {   //time and distance into seperate strings
    unoChar = (char)accString[stringCnt];                     //read character from unoString
      if(unoChar == ' ') {                //space char between time and distance string
        unoTimeBool = false;              //switch to distance string
        unoTime[stringCnt] = '\n';      //end of string unoTime
        stringCntOffset = stringCnt;
        unoChar = '0';
      }
      if(unoTimeBool) {
        unoTime[stringCnt] = unoChar;
      }
     else {
       unoDist[stringCnt - stringCntOffset] = unoChar;
      }
      stringCnt++;
    }
    unoDist[stringCnt - stringCntOffset] = '\n';           //end of string unoDist
    bufferString = unoTime;
    bufferString.trim();
    accUnoTime = accUnoTime + (int)bufferString.toInt();
    bufferString = unoDist;
    bufferString.trim();
    accUnoDist = accUnoDist + (int)bufferString.toInt();
    //Serial.print("Accumulated time = ");
    Serial.print(accUnoTime);
    Serial.print(" ");
    Serial.println(accUnoDist);
}  

/*-----------------------------------------------------------------------------------------------------
 * void remotePushCheck()
 * Reads remote push button on Arduino digital input.  Detects short push, long push, and double push.
 * Short push sends notification to tablet connected via Bluetooth.  Double push sends notification to
 * table connected via Bluetooth.  Long push stops motor via serial port connected to motor controller 
 * Arduino UNO.
 */
void remotePushCheck() {
  static long int pushStopTime = millis();
  static long int pushStartTime = pushStopTime;
  static long int doubleClickTime = pushStopTime;
  static bool pushStop = false;
  static bool pushStart = false;
  static bool doubleClick = false;
  
  bool buttonCondition = digitalRead(RemoteControl);      //read remote button
  long int pushOnTime = 0;
  
  if ((buttonCondition==false)&&(pushStart==false)) {     //detected remote button push
    pushStartTime = millis();                             //record time pushed
    pushStart = true;                                     //toggle push start
  }
  else if ((buttonCondition==true)&&(pushStart==true)) {  //detected remote button release
    pushStopTime = millis();                              //record time released
    pushOnTime = pushStopTime - pushStartTime;
    if(pushOnTime > 10) {                                 //debounce
      pushStop = true;
      pushStart = false;
    }
  }
  if(pushStart && (millis()-pushStartTime)>LONG_PUSH_TIME) {    //Remote Emergency Push Button STop
    Serial.println("REMOTE MOTOR STOP");
    Serial1.println("M0");
    pushStop = false;
    pushStart = false;
    doubleClick = false;
  }
  if((pushStop==true)) {                                  //check double push timer
    doubleClickTime = millis();                           //current time ms
    if((doubleClickTime-pushStopTime) > DOUBLE_CLICK_TIME) {            //check double click
      if(digitalRead(RemoteControl)==false) {             //is button pushed?
        Serial.println("REMOTE D");                       //button is pushed again during double click time
        Serial2.println("D");
        doubleClick = true;                               //button is still pushed so cancel immediate check for push
      }
      else {
        pushOnTime = pushStopTime - pushStartTime;        //time button has been pushed
        if(pushOnTime < LONG_PUSH_TIME) {                 //detect a short push
          if(doubleClick == false) {
            Serial.println("REMOTE S");
            Serial2.println("S");
          }
          doubleClick = false;
        }
      }
      pushStop = false;
    }
  }
}
