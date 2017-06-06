#include <Servo.h>
#include <SPI.h>
#include <i2c_t3.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055_t3.h"
#include "utility/imumaths.h"
#include "Actuators.h"
#include "Adafruit_MCP23017_t3.h"
Adafruit_MCP23017 mcp;
//Analog Multiplexor Select Pins
#define muxA 2
#define muxB 1
#define muxC 0

//Servo Pins
#define wrist 2
#define manip 11
#define base 12
//#define mast 14

#define rfJoyRX 14
#define rfJoyRY 15
#define rfJoyLX 17
#define rfJoyLY 16
#define rfSwitch1 20
#define rfSwitch2 21

volatile int pulseWidthRX;
volatile uint32_t pulseStartRX;
volatile int pulseWidthRY;
volatile uint32_t pulseStartRY;
volatile int pulseWidthLX;
volatile uint32_t pulseStartLX;
volatile int pulseWidthLY;
volatile uint32_t pulseStartLY;
volatile int pulseWidthSwitch1;
volatile uint32_t pulseStartSwitch1;
volatile int pulseWidthSwitch2;
volatile uint32_t pulseStartSwitch2;

const int manipClose = 125;
const int manipOpen = 75;

//Brushless Motors
Servo lfESC;
Servo rfESC;
Servo lrESC;
Servo rrESC;

/* Set the delay between fresh Gyroscope and acclerometer samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)
byte ADDRESS = 128;

// create servo objects to control a servo
Servo baseServo;
Servo manipulatorServo;
Servo wristServo;
//Servo mastServo;
//Servo tiltRServo;//uncomment for pan tilt

// default arm actuator positionts (TODO)
int elbowPosition = 1000;
int shoulderPosition = 1000;

// default servo positions
int basePosition = 1500;
int manipulatorPosition = 1500;
int clawPosition = 1;

int elbowVal = 0;

byte motorLeftSpeed = 0;
byte motorRightSpeed = 0;

boolean controlFlag = false;

int elbowSetPosition = 0;
int shoulderSetPosition = 0;
long previousMillis = 0;
bool rfController = false;

/*Create BNO055 object to access Gyro sensor reading*/
Adafruit_BNO055 bno = Adafruit_BNO055(55);//TODO PLAY WITH TIMEOUT IN LIBRARY

void setup()
{
  // Serial for communication with computer
  Serial.begin(115200);
  delay(2000);

  // Begin I/O expander (default address 0)
  mcp.begin();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Define OUTPUTs
  pinMode(elbowPWM, OUTPUT);
  pinMode(shoulderPWM, OUTPUT);
  mcp.pinMode(elbowDir, OUTPUT);
  mcp.pinMode(shoulderDir, OUTPUT);
  pinMode(leftRearPWM, OUTPUT);
  pinMode(rightRearPWM, OUTPUT);
  pinMode(leftFrontPWM, OUTPUT);
  pinMode(rightFrontPWM, OUTPUT);
  mcp.pinMode(leftRearIN1, OUTPUT);
  mcp.pinMode(leftRearIN2, OUTPUT);
  mcp.pinMode(rightRearIN1, OUTPUT);
  mcp.pinMode(rightRearIN2, OUTPUT);
  mcp.pinMode(leftFrontIN1, OUTPUT);
  mcp.pinMode(leftFrontIN2, OUTPUT);
  mcp.pinMode(rightFrontIN1, OUTPUT);
  mcp.pinMode(rightFrontIN2, OUTPUT);

  mcp.pinMode(muxA, OUTPUT);
  mcp.pinMode(muxB, OUTPUT);
  mcp.pinMode(muxC, OUTPUT);

  attachInterrupt(rfJoyLX, calcWidthLX, CHANGE);
  attachInterrupt(rfJoyLY, calcWidthLY, CHANGE);
  attachInterrupt(rfJoyRX, calcWidthRX, CHANGE);
  attachInterrupt(rfJoyRY, calcWidthRY, CHANGE);
  attachInterrupt(rfSwitch1, calcWidthSwitch1, CHANGE);
  attachInterrupt(rfSwitch2, calcWidthSwitch2, CHANGE);
  
  wristServo.attach(wrist, 600, 2400);
  //  for (int i = 0; i <
  wristServo.write(1200);
  delay(2000);

  baseServo.attach(base, 1100, 1900);  // attaches the servo on the base pin to the servo object, with limits of 1100-1900
  baseServo.write(1500);
  delay(500);
  // attaches the servo on the base pin to the servo object, with limits of 600-2400
  wristServo.write(1800);
  // attaches the servo on the base pin to the servo object, with limits of 500-1500
  manipulatorServo.attach(manip);
  manipulatorServo.write(manipOpen);


  //  panLServo.attach(panL);
  //  tiltLServo.attach(tiltL);
  //  panRServo.attach(panR);
  //  tiltRServo.attach(tiltR);//uncomment for pan/tilt

  //  panLServo.write(80);
  //  panRServo.write(80);
  //  tiltLServo.write(0);
  //  tiltRServo.write(0);//uncomment for pan/tilt

  //  mastServo.attach(mast);

  //attach ESC control pins to servo objects
  lfESC.attach(1);
  rfESC.attach(0);
  lrESC.attach(8);
  rrESC.attach(7);

  delay(100);
  lfESC.writeMicroseconds(0);
  rfESC.writeMicroseconds(0);
  lrESC.writeMicroseconds(0);
  rrESC.writeMicroseconds(0);

  // Move suspension to the minimum position (TODO: move to a starting midpoint position)
  // Move down
  setActuatorSpeed(leftFront, -100);
  setActuatorSpeed(rightFront, -100);
  setActuatorSpeed(leftRear, -100);
  setActuatorSpeed(rightRear, -100);
  //  setActuatorSpeed(leftFront, 200);
  //  setActuatorSpeed(rightFront, 200);
  //  setActuatorSpeed(leftRear, 200);
  //  setActuatorSpeed(rightRear, 200);
  // What for 5 seconds to ensure it is at the minimum
  delay(5000);

  //  // Stop
  setActuatorSpeed(leftFront, 0);
  setActuatorSpeed(rightFront, 0);
  setActuatorSpeed(leftRear, 0);
  setActuatorSpeed(rightRear, 0);
  //  delay(1000);
  //  setSuspensionPositions(map(0,0,100,33,929), map(0,0,100,79,942), map(0,0,100,92,977), map(0,0,100,61,912), 100);
  //  delay(2000);
  //  setActuatorSpeed(leftFront, 0);
  //  setActuatorSpeed(rightFront, 0);
  //  setActuatorSpeed(leftRear, 0);
  //  setActuatorSpeed(rightRear, 0);
  delay(2000);
  //  //set suspension to mid height
  //  Serial.println("Starting");
  //  setSuspensionPositions(map(100,0,100,33,929), map(100,0,100,79,942), map(100,0,100,92,977), map(100,0,100,61,912), 80);
  setActuatorSpeed(leftFront, 0);
  setActuatorSpeed(rightFront, 0);
  setActuatorSpeed(leftRear, 0);
  setActuatorSpeed(rightRear, 0);
  //  Serial.println("Done!");
  ////
  //  Serial.print("leftRear: ");Serial.println(analogReadMux(leftRearPos));
  //  Serial.print("leftFront: ");Serial.println(analogReadMux(leftFrontPos));
  //  Serial.print("rightRear: ");Serial.println(analogReadMux(rightRearPos));
  //  Serial.print("rightFront: ");Serial.println(analogReadMux(rightFrontPos));
  //  Serial.print("shoulder: ");Serial.println(analogReadMux(shoulderPos));
  //  Serial.print("elbow: ");Serial.println(analogReadMux(elbowPos));

  /* Initialise the BNO055 sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(1000);
    bno.begin();
  }
  bno.setExtCrystalUse(true);
  delay(1000);

}

void loop()
{
  const long interval = 3000; //ms to check for rf controller
//  unsigned long currentMillis = millis();
  //  int armSeconds = millis() + 20000;
  static int LF;
  static int RF;
  static int LB;
  static int RB;
  static int panLPos = 80;
  static int tiltLPos = 0;
  static int panRPos = 80;
  static int tiltRPos = 0;

  static int suspensionMode = 0;
  static int suspensionSpeed;
  static int timerStart = 0;
  static int stopTimerStart = 0;
  static bool timerOn = false;
  static bool stopTimerOn = false;
//  if(currentMillis - previousMillis > interval) {
//    // save the last time you blinked the LED 
//    previousMillis = currentMillis;
    if ((pulseWidthLX < 1100 && pulseWidthLX > 800) && !timerOn) {
      timerStart = millis();
      timerOn = true;
    }
    if (timerOn) {
      if (pulseWidthLX > 1100) {
        timerOn = false;
      }
      if ((millis() - timerStart > interval) && timerOn) {
        rfController = false;
        stopTimerOn = true;
        stopTimerStart = millis();
        timerOn = false;
      }
    }
    if (pulseWidthLX > 800 && (millis() - timerStart > interval + 3500)) {
      rfController = true;
    }
    if (millis() - stopTimerStart > 3000 && stopTimerOn) {
      stopTimerOn = false;
      pulseWidthLX = 0;
    }
//  }
  //  while (Serial.available() < 1) {
  //    if (milliSeconds <= millis()) {
  //      //motorsOff();
  //    }

  //    if (armSeconds <= millis()) {
  //      //elbowActuator.write(1000);   TODO  // sets the servo position according to the scaled value
  //      //shoulderActuator.write(1000); TODO // sets the servo position according to the scaled value
  //    }
  //  }


  
  
  //actuators
  //aLF,LB,RF,RB,MODE,speed
  //A0,1,0,1,1,200
  if (Serial.available() > 0) {
    char startChar = Serial.read();
    if (startChar == 'A') {
      if (!rfController) {
        LF = Serial.parseInt();
        LB = Serial.parseInt();
        RF = Serial.parseInt();
        RB = Serial.parseInt();
        suspensionMode = Serial.parseInt();
        suspensionSpeed = Serial.parseInt();
      }
      //    Serial.print("LF : "); Serial.println(LF);
      //    Serial.print("LB : "); Serial.println(LB);
      //    Serial.print("RF : "); Serial.println(RF);
      //    Serial.print("RB : "); Serial.println(RB);
      //    Serial.print("suspensionMode : "); Serial.println(suspensionMode);
      //    Serial.print("suspensionSpeed : "); Serial.println(suspensionSpeed);
    }
    else if (startChar == 'l' || startChar == 's') {
      char controlType = startChar;
      //Read from rover computer
      elbowPosition = Serial.parseInt();
      shoulderPosition = Serial.parseInt();
      basePosition = Serial.parseInt();
      manipulatorPosition = Serial.parseInt();
      clawPosition = Serial.parseInt();
      setMotors(true);
      setArmPositions(controlType);
      //    panLServo.write(panLPos);
      //    tiltLServo.write(tiltLPos);
      //    panRServo.write(panRPos);
      //    tiltRServo.write(tiltRPos);//uncomment for pan/tilt
    }
    else if (startChar == 'P') {
      //Ppan1,tilt1,pan2,tilt2,camera#(ignore)
      //TODO FIX JITTER
      panLPos = Serial.parseInt();
      tiltLPos = Serial.parseInt();
      panRPos = Serial.parseInt();
      tiltRPos = Serial.parseInt();
      //    panLServo.write(panLPos);
      //    tiltLServo.write(tiltLPos);
      //    panRServo.write(panRPos);
      //    tiltRServo.write(tiltRPos);//uncomment for pan/tilt
      Serial.parseInt();  //throw out camera#
    }
    else if (startChar == 'M') {
      static bool servoValue = false;
      servoValue = !servoValue;
      if (servoValue == false) {
        //      mastServo.write(20);
      }
      else {
        //      mastServo.write(120);
      }
    }
    if (!rfController)
    adjustSuspension(LF, LB, RF, RB, suspensionMode, suspensionSpeed);
  }
  else if (rfController){
    adjustSuspension(LF, LB, RF, RB, suspensionMode, suspensionSpeed);
    setMotors(false);
  }

}//END VOID LOOP

void setMotors(bool serial) {
  int leftValue;
  int rightValue;
  int leftValue2;
  int xVal;
  int yVal;
  int turboSwitch;
  bool turbo;
  int maxVal, minVal;
  if (serial) {
    motorRightSpeed = Serial.parseInt(); // 0 - 254
    motorLeftSpeed = Serial.parseInt();
  }
  if (rfController) {
    xVal = pulseWidthRX;
    yVal = pulseWidthRY;
//    Serial.println(pulseWidthRX);
    turboSwitch = pulseWidthSwitch1;
    if (turboSwitch < 1490) {
      turbo = false;
    }
    else {
      turbo = true;
    }

    if (xVal < 985 && yVal < 985) {
      motorRightSpeed = 127;
      motorLeftSpeed = 127;
    }
    else {
      if (turbo) {
        maxVal = 254;
        minVal = 0;
      }
      else {
        maxVal = 175;
        minVal = 79;
      }
      xVal = map(xVal, 990, 1979, minVal, maxVal);
      yVal = map(yVal, 990, 1979, minVal, maxVal);
      motorRightSpeed = yVal - xVal + 127;
      motorLeftSpeed = xVal + yVal - 127;
      if (motorLeftSpeed > maxVal)
        motorLeftSpeed = maxVal;
      if (motorLeftSpeed < minVal)
        motorLeftSpeed = minVal;
      if (motorRightSpeed > maxVal)
        motorRightSpeed = maxVal;
      if (motorRightSpeed < minVal)
        motorRightSpeed = minVal;
    }
  }

  rightValue = map(254 - motorRightSpeed, 0, 254, 1025, 1675);
  leftValue = map(motorLeftSpeed, 0, 254, 1025, 1675);//2000 = full fwd, 700 = full reverse  (set to half speed)
  leftValue2 = map(254 - motorLeftSpeed, 0, 254, 1025, 1675); //2000 = full fwd, 700 = full reverse  (set to half speed)
  //Indicates Serial has begun
  if (leftValue == 1350) {
    digitalWriteFast(13, HIGH);
  }

  //TODO Currently capped at half speed
  lfESC.writeMicroseconds(leftValue);
  rfESC.writeMicroseconds(rightValue);
  lrESC.writeMicroseconds(leftValue2);
  rrESC.writeMicroseconds(rightValue);
}

void setArmPositions(char controlType) {
  static int prevManipulatorPosition = 0;
  static int prevClawPosition = 0;
  //elbowActuator.write(elbowPosition);      TODO            // sets the servo position according to the scaled value
  //shoulderActuator.write(shoulderPosition);      TODO            // sets the servo position according to the scaled valu
  if (controlFlag && controlType == 's'  && (elbowPosition != 0 || shoulderPosition != 0)) {
    controlFlag = false;
  }
  if (controlType == 's') {
    if (!controlFlag) {
      int currentElbow = analogReadMux(elbowPos);
      int currentShoulder =  analogReadMux(shoulderPos);

      setActuatorSpeed(elbow, elbowPosition);
      setActuatorSpeed(shoulder, shoulderPosition);
    }
    else {
      int currentElbow = analogReadMux(elbowPos);
      int currentShoulder =  analogReadMux(shoulderPos);
      int elbowDiff = abs(currentElbow - elbowSetPosition);
      int shoulderDiff = abs(currentShoulder - shoulderSetPosition);
      if (elbowDiff < 10 && shoulderDiff < 10) {
        controlFlag = false;
        //TODO Send to mitch
      }
      else {
        if (elbowDiff >= 10) {
          if (currentElbow < elbowSetPosition) {
            setActuatorSpeed(elbow, 220);
          }
          else if (currentElbow > elbowSetPosition) {
            setActuatorSpeed(elbow, -220);
          }
        }
        else {
          setActuatorSpeed(elbow, 0);
        }
        if (shoulderDiff >= 10) {
          if (currentShoulder < shoulderSetPosition) {
            setActuatorSpeed(shoulder, 220);
          }
          else if (currentShoulder > shoulderSetPosition) {
            int speed = currentShoulder > 400 ? -220 : -220;
            setActuatorSpeed(shoulder, speed);
          }
        }
        else {
          setActuatorSpeed(shoulder, 0);
        }
      }
    }
  }
  else if (controlType == 'l') {
    controlFlag = true;
    elbowSetPosition = elbowPosition;
    shoulderSetPosition = shoulderPosition;
  }
  baseServo.write(basePosition);                  // sets the servo position according to the scaled value
  if (manipulatorPosition != prevManipulatorPosition) {
    wristServo.write(manipulatorPosition);
  }
  prevManipulatorPosition = manipulatorPosition;
  if (clawPosition != prevClawPosition) {
    if (clawPosition == 1) {
      manipulatorServo.write(manipClose);
    }
    else {
      manipulatorServo.write(manipOpen);
    }
  }
  prevClawPosition = clawPosition;
}

int analogReadMux(int pin) {
  mcp.digitalWrite(muxA, bitRead(pin, 0));
  mcp.digitalWrite(muxB, bitRead(pin, 1));
  mcp.digitalWrite(muxC, bitRead(pin, 2));
  return analogRead(A14);
}


/* This function controls the actuators for the arm and for the suspension, because they both work in the same way.
  The actuator names are part of the actuator enum, and are as follows:
  Suspension:
    leftFront
    leftRear
    rightFront
    rightRear
  Arm:
    leftElbow
    rightElbow
    leftShoulder
    rightShoulder

  The speed is a value between -255 and +255. The larger the magnitude the faster it moves. Negative values retract, positive extend, 0 is stopped.
*/
void setActuatorSpeed(actuator act, int speed2) {
  int suspensionMax = 1023;//TODO
  int suspensionMin = 0;//TODO
  int elbowMax = 635;
  int elbowMin = 20;
  int shoulderMax = 482;
  int shoulderMin = 20;
  //this should probably be a switch, but I always forget the syntax - Kyle
  if (act == leftRear) {
    if (speed2 > 0) {

      //if (analogReadMux(leftRearPos) < suspensionMax) {
      analogWrite(leftRearPWM, speed2);
      mcp.digitalWrite(leftRearIN1, HIGH);
      mcp.digitalWrite(leftRearIN2, LOW);
      //}
    }
    else if (speed2 < 0) {
      //if (analogReadMux(leftRearPos) > suspensionMin) {
      analogWrite(leftRearPWM, -speed2);
      mcp.digitalWrite(leftRearIN1, LOW);
      mcp.digitalWrite(leftRearIN2, HIGH);
      //}
    }
    else {
      analogWrite(leftRearPWM, speed2);
      mcp.digitalWrite(leftRearIN1, LOW);
      mcp.digitalWrite(leftRearIN2, LOW);
    }
  }
  else if (act == rightRear) {
    if (speed2 > 0) {

      //if (analogReadMux(rightRearPos) < suspensionMax) {
      analogWrite(rightRearPWM, speed2);
      mcp.digitalWrite(rightRearIN1, LOW);
      mcp.digitalWrite(rightRearIN2, HIGH);
      //}
    }
    else if (speed2 < 0) {
      //if (analogReadMux(rightRearPos) > suspensionMin) {
      analogWrite(rightRearPWM, -speed2);
      mcp.digitalWrite(rightRearIN1, HIGH);
      mcp.digitalWrite(rightRearIN2, LOW);
      //}
    }
    else {
      analogWrite(rightRearPWM, speed2);
      mcp.digitalWrite(rightRearIN1, LOW);
      mcp.digitalWrite(rightRearIN2, LOW);
    }
  }
  else if (act == leftFront) {
    if (speed2 > 0) {
      //if (analogReadMux(leftFrontPos) < suspensionMax) {
      analogWrite(leftFrontPWM, speed2);
      mcp.digitalWrite(leftFrontIN1, HIGH);
      mcp.digitalWrite(leftFrontIN2, LOW);
      //}
    }
    else if (speed2 < 0) {
      //if (analogReadMux(leftFrontPos) > suspensionMin) {
      analogWrite(leftFrontPWM, -speed2);
      mcp.digitalWrite(leftFrontIN1, LOW);
      mcp.digitalWrite(leftFrontIN2, HIGH);
      //}
    }
    else {
      analogWrite(leftFrontPWM, speed2);
      mcp.digitalWrite(leftFrontIN1, LOW);
      mcp.digitalWrite(leftFrontIN2, LOW);
    }
  }
  else if (act == rightFront) {
    if (speed2 > 0) {

      //if (analogReadMux(rightFrontPos) < suspensionMax) {
      analogWrite(rightFrontPWM, speed2);
      mcp.digitalWrite(rightFrontIN1, HIGH);
      mcp.digitalWrite(rightFrontIN2, LOW);
      //}
    }
    else if (speed2 < 0) {
      //if (analogReadMux(rightFrontPos) > suspensionMin) {
      analogWrite(rightFrontPWM, -speed2);
      mcp.digitalWrite(rightFrontIN1, LOW);
      mcp.digitalWrite(rightFrontIN2, HIGH);
      //}
    }
    else {
      analogWrite(rightFrontPWM, speed2);
      mcp.digitalWrite(rightFrontIN1, LOW);
      mcp.digitalWrite(rightFrontIN2, LOW);
    }
  }
  else if (act == elbow) {
    if (speed2 >= 0) {
      if (analogReadMux(elbowPos) < elbowMax) {
        analogWrite(elbowPWM, speed2);
        mcp.digitalWrite(elbowDir, LOW);
      }
      else {
        analogWrite(elbowPWM, 0);
        mcp.digitalWrite(elbowDir, LOW);
      }
    }
    else if (speed2 < 0) {
      if (analogReadMux(elbowPos) > elbowMin) {
        analogWrite(elbowPWM, -speed2);
        mcp.digitalWrite(elbowDir, HIGH);
      }
      else {
        analogWrite(elbowPWM, 0);
        mcp.digitalWrite(elbowDir, HIGH);
      }
    }
  }
  else if (act == shoulder) {
    if (speed2 >= 0) {
      if (analogReadMux(shoulderPos) < shoulderMax && speed2 != 0) {
        mcp.pinMode(shoulderDir, OUTPUT);
        analogWrite(shoulderPWM, speed2);
        mcp.digitalWrite(shoulderDir, LOW);
      }
      else {
        mcp.pinMode(shoulderDir, INPUT);
        analogWrite(shoulderPWM, 0);
        //        mcp.digitalWrite(shoulderDir, LOW);
      }
    }
    else if (speed2 < 0) {
      mcp.pinMode(shoulderDir, OUTPUT);
      if (analogReadMux(shoulderPos) > shoulderMin) {
        analogWrite(shoulderPWM, -speed2);
        mcp.digitalWrite(shoulderDir, HIGH);
      }
      else {
        analogWrite(shoulderPWM, 0);
        mcp.digitalWrite(shoulderDir, HIGH);
      }
    }
  }
}

void adjustSuspension(int LF, int LB, int RF, int RB, int mode, int speed) {
  float levelValuePitch = 0;
  float levelValueRoll = 0;
  float pitch;
  float roll;
  float pitchErrorP;
  float rollErrorP;
  int pitchTotalError;
  int rollTotalError;
  int LFError;
  int RFError;
  int LRError;
  int RRError;
  int Kp = 30;
  int minPWM = 60;
  bool pitchCorrect = 0;
  int xVal;
  int yVal;
  sensors_event_t event;
  bno.getEvent(&event);
  //  pitchCorrect = clawPosition;
  pitch = -event.orientation.y;
  roll = event.orientation.z;
  if (roll < 0) {
    roll = -(180 + roll);
  }
  else {
    roll = 180 - roll;
  }
  pitchErrorP = pitch - levelValuePitch;
  rollErrorP = roll - levelValueRoll;
  //  Serial.println(pitchErrorP);
  int LFDiff = (1 * (map(50, 0, 100, 61, 912) - (1024 - analogReadMux(leftFrontPos))));
  int RFDiff = (1 * (map(50, 0, 100, 33, 929) - (1024 - analogReadMux(rightFrontPos))));
  int LRDiff = (1 * (map(50, 0, 100, 92, 977) - (1024 - analogReadMux(leftRearPos))));
  int RRDiff = (1 * (map(50, 0, 100, 79, 942) - (1024 - analogReadMux(rightRearPos))));

  const int correctionThres = 5;
  if (pitchErrorP > correctionThres || rollErrorP > correctionThres) {
    Kp = 10;
  }
  else {
    Kp = 30;
  }
  int avgDiff = (LFDiff + RFDiff + LRDiff + RRDiff) / 4;
  //avgDiff = 0;
  LFError = Kp * (pitchErrorP - rollErrorP) + avgDiff;
  RFError = Kp * (pitchErrorP + rollErrorP) + avgDiff;
  LRError = Kp * (-pitchErrorP - rollErrorP) + avgDiff;
  RRError = Kp * (-pitchErrorP + rollErrorP) + avgDiff;


  if (LFError >= minPWM || LFError <= -minPWM) {
    //    LFError *= Kp;
  }
  else {
    LFError = 0;
  }
  if (RFError >= minPWM || RFError <= -minPWM) {
    //    RFError *= Kp;
  }
  else {
    RFError = 0;
  }
  if (LRError >= minPWM || LRError <= -minPWM) {
    //    LRError *= Kp;
  }
  else {
    LRError = 0;
  }
  if (RRError >= minPWM || RRError <= -minPWM) {
    //    RRError = RRError *= Kp;
  }
  else {
    RRError = 0;
  }
  //  Serial.print("LF: "); Serial.print(LFError); Serial.print("  "); Serial.print("RF: "); Serial.println(RFError);
  //  Serial.print("LR: "); Serial.print(LRError); Serial.print("  "); Serial.print("RR: "); Serial.println(RRError);
  if (rfController) {
    int switchVal = pulseWidthSwitch2;
    xVal = pulseWidthLX;
    yVal = pulseWidthLY;
    if (switchVal < 1490) {
      mode = 0;
    }
    else {
      mode = 1;
    }
    //    Serial.print("xVal: "); Serial.print(xVal); Serial.print("   yVal: "); Serial.println(yVal);
    //    Serial.println(switchVal);
  }

  if (mode == 0) {
    setActuatorSpeed(leftFront, LFError);
    setActuatorSpeed(rightFront, RFError);
    setActuatorSpeed(leftRear, LRError);
    setActuatorSpeed(rightRear, RRError);
  }
  else {
    if (rfController) {
      if (xVal > 1350 && xVal < 1650) {
        speed = map(yVal, 990, 1979, -250, 250);
        if (speed > -30 && speed < 30) {
          speed = 0;
        }
        setActuatorSpeed(leftFront, speed);
        setActuatorSpeed(rightFront, speed);
        setActuatorSpeed(leftRear, speed);
        setActuatorSpeed(rightRear, speed);
      }
    }
    else {
      setActuatorSpeed(leftFront, speed * LF);
      setActuatorSpeed(rightFront, speed * RF);
      setActuatorSpeed(leftRear, speed * LB);
      setActuatorSpeed(rightRear, speed * RB);
    }
  }
  Serial.print(pitch);//pitch
  Serial.print(",");
  Serial.println(roll);//roll
  Serial.flush();
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setSuspensionPositions(int RF, int RR, int LR, int LF, int speed) {//blocking
  int allSet = false;
  int threshold = 20;
  int timeout = 1000000 / speed; //ms
  int RFSpeed;
  int RRSpeed;
  int LRSpeed;
  int LFSpeed;
  bool RFSet = false;
  bool RRSet = false;
  bool LRSet = false;
  bool LFSet = false;
  long startTime = millis();
  if (analogReadMux(rightFrontPos) > RF + threshold) {
    RFSpeed = -abs(speed);//speed should be positive, but just in case
  }
  else if (analogReadMux(rightFrontPos) < RF - threshold) {
    RFSpeed = 2 * abs(speed);
  }
  else {
    RFSpeed = 0;
  }
  if (analogReadMux(rightRearPos) > RR + threshold) {
    RRSpeed = -abs(speed);//speed should be positive, but just in case
  }
  else if (analogReadMux(rightRearPos) < RR - threshold) {
    RRSpeed = 2 * abs(speed);
  }
  else {
    RRSpeed = 0;
  }
  if (analogReadMux(leftRearPos) > LR + threshold) {
    LRSpeed = -abs(speed);//speed should be positive, but just in case
  }
  else if (analogReadMux(leftRearPos) < LR - threshold) {
    LRSpeed = 2 * abs(speed);
  }
  else {
    LRSpeed = 0;
  }
  if (analogReadMux(leftFrontPos) > LF + threshold) {
    LFSpeed = -abs(speed);//speed should be positive, but just in case
  }
  else if (analogReadMux(leftFrontPos) < LF - threshold) {
    LFSpeed = 2 * abs(speed);
  }
  else {
    LFSpeed = 0;
  }
  //  Serial.println(allSet);
  while (allSet == false) {
    int RFPos = analogReadMux(rightFrontPos);
    int RRPos = analogReadMux(rightRearPos);
    int LRPos = analogReadMux(leftRearPos);
    int LFPos = analogReadMux(leftFrontPos);
    if (RFPos <= RF + threshold && RFPos >= RF - threshold) {
      RFSet = true;
      //      Serial.println("RF");
      setActuatorSpeed(rightFront, 0);
    }
    else {
      setActuatorSpeed(rightFront, RFSpeed);
    }
    if (RRPos <= RR + threshold && RRPos >= RR - threshold) {
      RRSet = true;
      //      Serial.println("RR");
      setActuatorSpeed(rightRear, 0);
    }
    else {
      setActuatorSpeed(rightRear, RRSpeed);
    }
    if (LFPos <= LF + threshold && LFPos >= LF - threshold) {
      LFSet = true;
      //     Serial.println("LF");
      setActuatorSpeed(leftFront, 0);
    }
    else {
      setActuatorSpeed(leftFront, LFSpeed);
    }
    if (LRPos <= LR + threshold && LRPos >= LR - threshold) {
      LRSet = true;
      //      Serial.println("LR");
      setActuatorSpeed(leftRear, 0);
    }
    else {
      setActuatorSpeed(leftRear, LRSpeed);
    }
    if (LFSet == true && RFSet == true && LRSet == true && RRSet == true) {
      allSet = true;
    }
    if (millis() - startTime > timeout) {
      allSet = true;
      setActuatorSpeed(rightFront, 0);
      setActuatorSpeed(rightRear, 0);
      setActuatorSpeed(leftRear, 0);
      setActuatorSpeed(leftFront, 0);
    }
  }
}

//http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html
void calcWidthRX()
{
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if(digitalReadFast(rfJoyRX) == HIGH)
  { 
    pulseStartRX = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling 
    // edges i.e. the pulse duration.
    pulseWidthRX = (uint16_t)(micros() - pulseStartRX);
  }
}

void calcWidthRY()
{
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if(digitalReadFast(rfJoyRY) == HIGH)
  { 
    pulseStartRY = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling 
    // edges i.e. the pulse duration.
    pulseWidthRY = (uint16_t)(micros() - pulseStartRY);
  }
}

void calcWidthLX()
{
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if(digitalReadFast(rfJoyLX) == HIGH)
  { 
    pulseStartLX = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling 
    // edges i.e. the pulse duration.
    pulseWidthLX = (uint16_t)(micros() - pulseStartLX);
  }
}

void calcWidthLY()
{
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if(digitalReadFast(rfJoyLY) == HIGH)
  { 
    pulseStartLY = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling 
    // edges i.e. the pulse duration.
    pulseWidthLY = (uint16_t)(micros() - pulseStartLY);
  }
}

void calcWidthSwitch1()
{
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if(digitalReadFast(rfSwitch1) == HIGH)
  { 
    pulseStartSwitch1 = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling 
    // edges i.e. the pulse duration.
    pulseWidthSwitch1 = (uint16_t)(micros() - pulseStartSwitch1);
  }
}

void calcWidthSwitch2()
{
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if(digitalReadFast(rfSwitch2) == HIGH)
  { 
    pulseStartSwitch2 = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling 
    // edges i.e. the pulse duration.
    pulseWidthSwitch2 = (uint16_t)(micros() - pulseStartSwitch2);
  }
}
