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

#define muxA 2
#define muxB 1
#define muxC 0

#define camera 16
#define wrist 2
#define manip 11
#define base 12


#define LEFTMOTOR Serial1
#define RIGHTMOTOR Serial3
/* Set the delay between fresh Gyroscope and acclerometer samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
byte ADDRESS = 128;

Servo baseServo;  // create servo object to control a servo
Servo manipulatorServo;
Servo clawServo;

int elbowPosition = 1000; // default positionts.
int shoulderPosition = 1000;

int basePosition = 1500;

int manipulatorPosition = 1500;
int clawPosition = 1;

int elbowVal = 0;

byte motorLeftSpeed = 0;
byte motorRightSpeed = 0;

 /*Create BNO055 object to access Gyro sensor reading*/
Adafruit_BNO055 bno = Adafruit_BNO055(55);
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
void setup()
{

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the BNO055 sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

    /* Display some basic information on this sensor */
  displaySensorDetails();

  mcp.begin();      // use default address 0
  pinMode(13, OUTPUT);
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

//  while(1) {
//    digitalWrite(leftRearPWM, HIGH);
//    delay(1000);
//    digitalWrite(leftRearPWM, LOW);
//    delay(1000);
//  }
  setActuatorSpeed(leftRear, 80);
  delay(1000);
  setActuatorSpeed(leftRear, 0);
  delay(500);
  setActuatorSpeed(leftRear, -80);
  delay(1000);
  setActuatorSpeed(leftRear, 0);
  delay(1000000000);
  
  //shoulderActuator.write(1000); TODO
  //delay(7000);

  //elbowActuator.write(1000);TODO
  delay(2000);

  baseServo.attach(12, 1100, 1900);  // attaches the servo on pin 9 to the servo object
  baseServo.write(1500);

  manipulatorServo.attach(2, 600, 2400);  // attaches the servo on pin 9 to the servo object
  clawServo.attach(11, 500, 1500);  // attaches the servo on pin 9 to the servo object
  
  LEFTMOTOR.begin(38400);
  RIGHTMOTOR.begin(38400);

}

void loop()
{
  
   /* Get a new BNO055 sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  float prev_pitchValue;
  
  int milliSeconds = millis() + 1000;
  int armSeconds = millis() + 20000;

  while (Serial.available() < 1) {//waits for computer to start TODO: try in setup
    if (milliSeconds <= millis()) {
      motorsOff();
    }

    if (armSeconds <= millis()) {
      //elbowActuator.write(1000);   TODO  // sets the servo position according to the scaled value
      //shoulderActuator.write(1000); TODO // sets the servo position according to the scaled value
    }
  }

  if (Serial.read() == 's') 
  {
    //Read from rover computer
    elbowPosition = Serial.parseInt();
    shoulderPosition = Serial.parseInt();
    basePosition = Serial.parseInt();
    manipulatorPosition = Serial.parseInt();
    clawPosition = Serial.parseInt();
   
    setMotors();

    if ((event.orientation.y > 10) || (event.orientation.y <-10))
    {
      /*Active Suspension needed*/
      if(prev_pitchValue < event.orientation.y)
      {
        setActuatorSpeed(leftRear, 40);
        setActuatorSpeed(rightRear, 40);
        Serial.print("Raise Rear Wheels");
        Serial.println(F(""));
      }
      else
      {
        setActuatorSpeed(leftFront, 40);
        setActuatorSpeed(rightFront, 40);
        Serial.print("Raise Front Wheels");
        Serial.println(F(""));
      }
    }
    else
    {
      /*Active Suspension Not needed*/
    }
    prev_pitchValue = event.orientation.y;
    Serial.print(F("\n"));
    Serial.print((float)event.orientation.x);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));
  }

  setArmPositions();
  
}//END VOID LOOP


int checksum(int one, int two, int three) {
  return (one + two + three) & 0x7F;
}

void motorsOff() {
  RIGHTMOTOR.write(ADDRESS);
  RIGHTMOTOR.write(0); // command
  RIGHTMOTOR.write(0);
  RIGHTMOTOR.write(checksum(ADDRESS, 0, 0));

  RIGHTMOTOR.write(ADDRESS);
  RIGHTMOTOR.write(4); // command
  RIGHTMOTOR.write(0);
  RIGHTMOTOR.write(checksum(ADDRESS, 4, 0));

  LEFTMOTOR.write(ADDRESS);
  LEFTMOTOR.write(0); // command
  LEFTMOTOR.write(0);
  LEFTMOTOR.write(checksum(ADDRESS, 0, 0));

  LEFTMOTOR.write(ADDRESS);
  LEFTMOTOR.write(4); // command
  LEFTMOTOR.write(0);
  LEFTMOTOR.write(checksum(ADDRESS, 4, 0));
}

void setMotors() {
  motorRightSpeed = Serial.parseInt(); // -127 - 127

  byte command;
  byte command2;

  if (motorRightSpeed >= 127) {
    command = 1;
    command2 = 5;
    motorRightSpeed -= 127;
  }
  else {
    command = 0;
    command2 = 4;
    motorRightSpeed = 127 - motorRightSpeed;
  }

  RIGHTMOTOR.write(ADDRESS);
  RIGHTMOTOR.write(command); // command
  RIGHTMOTOR.write(abs(motorRightSpeed));
  RIGHTMOTOR.write(checksum(ADDRESS, command, abs(motorRightSpeed)));

  RIGHTMOTOR.write(ADDRESS);
  RIGHTMOTOR.write(command2); // command
  RIGHTMOTOR.write(abs(motorRightSpeed));
  RIGHTMOTOR.write(checksum(ADDRESS, command2, abs(motorRightSpeed)));

  RIGHTMOTOR.write(ADDRESS);
  RIGHTMOTOR.write(49); // Read Motor Currents

  int16_t rightMotor1;
  int16_t rightMotor2;

  rightMotor1 = 0;
  rightMotor2 = 0;

  rightMotor1 += RIGHTMOTOR.read();
  rightMotor1  = rightMotor1 << 8;
  rightMotor1 += RIGHTMOTOR.read();
  rightMotor2 += RIGHTMOTOR.read();
  rightMotor2  = rightMotor2 << 8;
  rightMotor2 += RIGHTMOTOR.read();

  RIGHTMOTOR.clear();

  motorLeftSpeed = Serial.parseInt();

  if (motorLeftSpeed >= 127) {
    command = 1;
    command2 = 5;
    motorLeftSpeed -= 127;
  }
  else {
    command = 0;
    command2 = 4;
    motorLeftSpeed = 127 - motorLeftSpeed;
  }

  LEFTMOTOR.write(ADDRESS);
  LEFTMOTOR.write(command); // command
  LEFTMOTOR.write(abs(motorLeftSpeed));
  LEFTMOTOR.write(checksum(ADDRESS, command, abs(motorLeftSpeed)));

  LEFTMOTOR.write(ADDRESS);
  LEFTMOTOR.write(command2); // command
  LEFTMOTOR.write(abs(motorLeftSpeed));
  LEFTMOTOR.write(checksum(ADDRESS, command2, abs(motorLeftSpeed)));

  LEFTMOTOR.write(ADDRESS);
  LEFTMOTOR.write(49); // Read Motor Currents

  int16_t leftMotor1 = 0;
  int16_t leftMotor2 = 0;

  leftMotor1 += LEFTMOTOR.read();
  leftMotor1  = leftMotor1 << 8;
  leftMotor1 += LEFTMOTOR.read();
  leftMotor2 += LEFTMOTOR.read();
  leftMotor2  = leftMotor2 << 8;
  leftMotor2 += LEFTMOTOR.read();

  LEFTMOTOR.clear();

  Serial.print("d");
  Serial.print((double)rightMotor1 / 100.0);
  Serial.print(",");
  Serial.print((double)rightMotor2 / 100.0);
  Serial.print(",");
  Serial.print((double)leftMotor1 / 100.0);
  Serial.print(",");
  Serial.print((double)leftMotor2 / 100.0);
  Serial.print(",");
}

void setArmPositions() {
  //elbowActuator.write(elbowPosition);      TODO            // sets the servo position according to the scaled value
  //shoulderActuator.write(shoulderPosition);      TODO            // sets the servo position according to the scaled value
  setActuatorSpeed(leftElbow, elbowPosition);
  setActuatorSpeed(rightElbow, elbowPosition);
  setActuatorSpeed(leftShoulder, shoulderPosition);
  setActuatorSpeed(rightShoulder, shoulderPosition);
  
  baseServo.write(basePosition);                  // sets the servo position according to the scaled value
  manipulatorServo.write(manipulatorPosition);

  if (clawPosition == 1) {
    clawServo.write(1250); // 940
  }
  else {
    clawServo.write(1550); // 940
  }
}

int analogReadMux(int pin) { 
  mcp.digitalWrite(muxA, bitRead(pin, 0));
  mcp.digitalWrite(muxB, bitRead(pin, 1));
  mcp.digitalWrite(muxC, bitRead(pin, 2));
  return analogRead(A14);
}

void setActuatorSpeed(actuator act, int speed2) {
  int suspensionMax = 1023;//TODO
  int suspensionMin = 0;//TODO
  int armMax = 1000;
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

