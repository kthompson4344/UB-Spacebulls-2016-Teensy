/*
 Controlling a servo position using a potentiometer (variable resistor)
 */

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Actuators.h"
#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcp;

//Analog Multiplexor Select Pins
#define muxA 2
#define muxB 1
#define muxC 0

//Servo Pins
#define camera 16
#define wrist 2
#define manip 11
#define base 12

//Brushless Motors
Servo lfESC;

/* Set the delay between fresh Gyroscope and acclerometer samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
byte ADDRESS = 128;

// create servo objects to control a servo
Servo baseServo;
Servo manipulatorServo;
Servo clawServo;

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

/*Create BNO055 object to access Gyro sensor reading*/
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup()
{
  delay(2000);

  // Serial for communication with computer
  Serial.begin(115200);

  /* Initialise the BNO055 sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  // Begin I/O expander (default address 0)
  mcp.begin();

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Define OUTPUTs
  pinMode(leftElbowPWM, OUTPUT);
  pinMode(rightElbowPWM, OUTPUT);
  pinMode(leftShoulderPWM, OUTPUT);
  pinMode(rightShoulderPWM, OUTPUT);

  mcp.pinMode(leftElbowDir, OUTPUT);
  mcp.pinMode(rightElbowDir, OUTPUT);
  mcp.pinMode(leftShoulderDir, OUTPUT);
  mcp.pinMode(rightShoulderDir, OUTPUT);

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

  //TODO: Elbow and Shoulder to starting positions

  baseServo.attach(base, 1100, 1900);  // attaches the servo on the base pin to the servo object, with limits of 1100-1900
  baseServo.write(1500);

  // attaches the servo on the base pin to the servo object, with limits of 600-2400
  manipulatorServo.attach(wrist, 600, 2400);
  // attaches the servo on the base pin to the servo object, with limits of 500-1500
  clawServo.attach(16, 500, 1500);

  //attach ESC control pins to servo objects
  lfESC.attach(0);
//  rfESC.attach();



  delay(100);
  lfESC.writeMicroseconds(0);  

  // Move suspension to the minimum position (TODO: move to a starting midpoint position)
  // Move down
  setActuatorSpeed(leftFront, -100);
  setActuatorSpeed(rightFront, -100);
  setActuatorSpeed(leftRear, -100);
  setActuatorSpeed(rightRear, -100);
  // What for 5 seconds to ensure it is at the minimum
  delay(5000);
  // Stop
  setActuatorSpeed(leftFront, 0);
  setActuatorSpeed(rightFront, 0);
  setActuatorSpeed(leftRear, 0);
  setActuatorSpeed(rightRear, 0);
}

void loop()
{

  int milliSeconds = millis() + 1000;
  int armSeconds = millis() + 20000;

  while (Serial.available() < 1) {//waits for computer to start TODO: try in setup
    if (milliSeconds <= millis()) {
      //motorsOff();
    }

    if (armSeconds <= millis()) {
      //elbowActuator.write(1000);   TODO  // sets the servo position according to the scaled value
      //shoulderActuator.write(1000); TODO // sets the servo position according to the scaled value
    }
  }


  char controlType = Serial.read();
  //Read from rover computer
  elbowPosition = Serial.parseInt();
  shoulderPosition = Serial.parseInt();
  basePosition = Serial.parseInt();
  manipulatorPosition = Serial.parseInt();
  clawPosition = Serial.parseInt();
  setMotors();
  setArmPositions(controlType);
  adjustSuspension();


  //setArmPositions() used to be here?
}//END VOID LOOP


int checksum(int one, int two, int three) {
  return (one + two + three) & 0x7F;
}

void motorsOff() {
}

void setMotors() {
  int leftValue;
  int rightValue;
  motorRightSpeed = Serial.parseInt(); // -127 - 127
  motorLeftSpeed = Serial.parseInt();

  rightValue = map(motorRightSpeed, 0, 254, 700, 2000);
  leftValue = map(motorLeftSpeed, 0, 254, 700, 2000);
  if (leftValue == 1350) {
    digitalWriteFast(13, HIGH);
  }
  lfESC.writeMicroseconds(leftValue);
  
  //TODO other motors
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
      int currentElbow = analogReadMux(leftElbowPos);
      int currentShoulder =  analogReadMux(rightShoulderPos);
      Serial.println("Current: " + (String)currentElbow + " " + (String)currentShoulder);
      Serial.flush();

      setActuatorSpeed(leftElbow, elbowPosition);
      setActuatorSpeed(rightElbow, elbowPosition);
      setActuatorSpeed(leftShoulder, shoulderPosition);
      setActuatorSpeed(rightShoulder, shoulderPosition);
    }
    else {
      int currentElbow = analogReadMux(leftElbowPos);
      int currentShoulder =  analogReadMux(rightShoulderPos);
      Serial.println("Expected: " + (String)elbowSetPosition + " " + (String)shoulderSetPosition);
      Serial.println("Current: " + (String)currentElbow + " " + (String)currentShoulder);
      Serial.flush();
      int elbowDiff = abs(currentElbow - elbowSetPosition);
      int shoulderDiff = abs(currentShoulder - shoulderSetPosition);
      if (elbowDiff < 10 && shoulderDiff < 10) {
        controlFlag = false;
      }
      else {
        if (elbowDiff >= 10) {
          if (currentElbow < elbowSetPosition) {
            setActuatorSpeed(leftElbow, 127);
            setActuatorSpeed(rightElbow, 127);
          }
          else if (currentElbow > elbowSetPosition) {
            setActuatorSpeed(leftElbow, -127);
            setActuatorSpeed(rightElbow, -127);
          }
        }
        else {
          setActuatorSpeed(leftElbow, 0);
        }
        if (shoulderDiff >= 10) {
          if (currentShoulder < shoulderSetPosition) {
            setActuatorSpeed(leftShoulder, 127);
            setActuatorSpeed(rightShoulder, 127);
          }
          else if (currentShoulder > shoulderSetPosition) {
            setActuatorSpeed(leftShoulder, -127);
            setActuatorSpeed(rightShoulder, -127);
          }
        }
        else {
          setActuatorSpeed(leftShoulder, 0);
          setActuatorSpeed(rightShoulder, 0);
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
    manipulatorServo.write(manipulatorPosition);
  }
  prevManipulatorPosition = manipulatorPosition;
  if (clawPosition != prevClawPosition) {
    if (clawPosition == 1) {
      clawServo.write(1250); // 940
    }
    else {
      clawServo.write(1550); // 940
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
  int armMax = 900;
  int armMin = 20;
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
      mcp.digitalWrite(leftFrontIN1, LOW);
      mcp.digitalWrite(leftFrontIN2, HIGH);
      //}
    }
    else if (speed2 < 0) {
      //if (analogReadMux(leftFrontPos) > suspensionMin) {
      analogWrite(leftFrontPWM, -speed2);
      mcp.digitalWrite(leftFrontIN1, HIGH);
      mcp.digitalWrite(leftFrontIN2, LOW);
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
  else if (act == leftElbow) {
    if (speed2 >= 0) {
      if (analogReadMux(leftElbowPos) < armMax) {
        analogWrite(leftElbowPWM, speed2);
        mcp.digitalWrite(leftElbowDir, LOW);
      }
      else {
        analogWrite(leftElbowPWM, 0);
        mcp.digitalWrite(leftElbowDir, LOW);
      }
    }
    else if (speed2 < 0) {
      if (analogReadMux(leftElbowPos) > armMin) {
        analogWrite(leftElbowPWM, -speed2);
        mcp.digitalWrite(leftElbowDir, HIGH);
      }
      else {
        analogWrite(leftElbowPWM, 0);
        mcp.digitalWrite(leftElbowDir, HIGH);
      }
    }
  }
  else if (act == rightElbow) {
    if (speed2 >= 0) {
      if (analogReadMux(rightElbowPos) < armMax) {
        analogWrite(rightElbowPWM, speed2);
        mcp.digitalWrite(rightElbowDir, LOW);
      }
      else {
        analogWrite(rightElbowPWM, 0);
        mcp.digitalWrite(rightElbowDir, LOW);
      }
    }
    else if (speed2 < 0) {
      if (analogReadMux(rightElbowPos) > armMin) {
        analogWrite(rightElbowPWM, -speed2);
        mcp.digitalWrite(rightElbowDir, HIGH);
      }
      else {
        analogWrite(rightElbowPWM, 0);
        mcp.digitalWrite(rightElbowDir, HIGH);
      }
    }
  }
  else if (act == leftShoulder) {
    if (speed2 >= 0) {
      if (analogReadMux(leftShoulderPos) < armMax) {
        analogWrite(leftShoulderPWM, speed2);
        mcp.digitalWrite(leftShoulderDir, LOW);
      }
      else {
        analogWrite(leftShoulderPWM, 0);
        mcp.digitalWrite(leftShoulderDir, LOW);
      }
    }
    else if (speed2 < 0) {
      if (analogReadMux(leftShoulderPos) > armMin) {
        analogWrite(leftShoulderPWM, -speed2);
        mcp.digitalWrite(leftShoulderDir, HIGH);
      }
      else {
        analogWrite(leftShoulderPWM, 0);
        mcp.digitalWrite(leftShoulderDir, HIGH);
      }
    }
  }
  else if (act == rightShoulder) {
    if (speed2 >= 0) {
      if (analogReadMux(rightShoulderPos) < armMax) {
        analogWrite(rightShoulderPWM, speed2);
        mcp.digitalWrite(rightShoulderDir, LOW);
      }
      else {
        analogWrite(rightShoulderPWM, 0);
        mcp.digitalWrite(rightShoulderDir, LOW);
      }
    }
    else if (speed2 < 0) {
      if (analogReadMux(rightShoulderPos) > armMin) {
        analogWrite(rightShoulderPWM, -speed2);
        mcp.digitalWrite(rightShoulderDir, HIGH);
      }
      else {
        analogWrite(rightShoulderPWM, 0);
        mcp.digitalWrite(rightShoulderDir, HIGH);
      }
    }
  }

}

void adjustSuspension() {
  float errorP;
  int totalError;
  int Kp = 50;
  bool pitchCorrect;
  float levelValuePitch = 6.6;
  float levelValueRoll = 3.3;
  float pitch;
  float roll;
  //Serial.println("beginning");
  sensors_event_t event;
  bno.getEvent(&event);
  pitchCorrect = clawPosition;
  pitch = event.orientation.y;
  roll = event.orientation.z;
  if (pitchCorrect == 1) {
    errorP = pitch - levelValuePitch;
    totalError = Kp * errorP;
    if (totalError >= 255) {
      totalError = 255;
    }
    if (totalError <= -255) {
      totalError = -255;
    }
    //Serial.println(totalError);
    if (totalError >= 40 || totalError <= -40) {
      setActuatorSpeed(leftFront, totalError);
      setActuatorSpeed(rightFront, totalError);
      setActuatorSpeed(leftRear, -totalError);
      setActuatorSpeed(rightRear, -totalError);
    }
    else {
      setActuatorSpeed(leftFront, 0);
      setActuatorSpeed(rightFront, 0);
      setActuatorSpeed(leftRear, 0);
      setActuatorSpeed(rightRear, 0);
    }
  }
  else {
    //errorP = levelValueRoll - event.orientation.z;
    errorP = roll - levelValueRoll;
    totalError = Kp * errorP;
    if (totalError >= 255) {
      totalError = 255;
    }
    if (totalError <= -255) {
      totalError = -255;
    }
    //Serial.println(totalError);
    if (totalError >= 45 || totalError <= -45) {
      setActuatorSpeed(leftFront, -totalError);
      setActuatorSpeed(rightFront, totalError);
      setActuatorSpeed(leftRear, -totalError);
      setActuatorSpeed(rightRear, totalError);
    }
    else {
      setActuatorSpeed(leftFront, 0);
      setActuatorSpeed(rightFront, 0);
      setActuatorSpeed(leftRear, 0);
      setActuatorSpeed(rightRear, 0);
    }
  }
  /*
  Serial.print(pitch);//pitch
  Serial.print(",");
  Serial.print(roll);//roll
  Serial.println(",");
  Serial.flush();
  */
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

