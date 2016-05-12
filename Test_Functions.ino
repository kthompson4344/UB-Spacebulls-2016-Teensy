void printAnalog() {
  Serial.print("leftFrontPos: "); Serial.println(analogReadMux(leftFrontPos));
  Serial.print("rightFrontPos: "); Serial.println(analogReadMux(rightFrontPos));
  Serial.print("leftRearPos: "); Serial.println(analogReadMux(leftRearPos));
  Serial.print("rightRearPos: "); Serial.println(analogReadMux(rightRearPos));
  Serial.print("shoulderPos: "); Serial.println(analogReadMux(shoulderPos));
  Serial.print("elbowPos: "); Serial.println(analogReadMux(elbowPos));
}

void testServos() {
  //Need to apply 6V to the +6V connector
  int pos;
  for (pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees
  { // in steps of 1 degree
    baseServo.write(pos);              // tell servo to go to position in variable 'pos'
    wristServo.write(pos);
    manipulatorServo.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) // goes from 180 degrees to 0 degrees
  {
    baseServo.write(pos);              // tell servo to go to position in variable 'pos'
    wristServo.write(pos);
    manipulatorServo.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void testActuators() {
  const int interval = 1000;
  static int ledState = LOW;
  static long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {

    // save the last time you blinked the LED
    previousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
      setActuatorSpeed(leftFront, 100);
      setActuatorSpeed(rightFront, 100);
      setActuatorSpeed(leftRear, 100);
      setActuatorSpeed(rightRear, 100);
//      setActuatorSpeed(shoulder, 200);
//      setActuatorSpeed(elbow, 100);
    }
    else {
      ledState = LOW;
      setActuatorSpeed(leftFront, -100);
      setActuatorSpeed(rightFront, -100);
      setActuatorSpeed(leftRear, -100);
      setActuatorSpeed(rightRear, -100);
//      setActuatorSpeed(shoulder, -200);
//      setActuatorSpeed(elbow, -100);
    }
  }
}

void printAngle() {
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("x: "); Serial.println(event.orientation.x);
  Serial.print("y: "); Serial.println(event.orientation.y);
  Serial.print("z: "); Serial.println(event.orientation.z);
}



