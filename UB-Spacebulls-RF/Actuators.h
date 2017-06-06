/*
 * Backup rover
 * 
 * min:
 * leftRear: 92
 * leftFront: 61
 * rightRear: 79
 * rightFront: 33
 * 
 * max:
 * leftRear: 977
 * leftFront: 912
 * rightRear: 942
 * rightFront: 929
 */


enum actuator{
  leftFront = 0,
  leftRear = 1,
  rightFront = 2,
  rightRear = 3,
  elbow = 4,
  shoulder = 5,
};

#define elbowPWM 22
#define shoulderPWM 23

#define elbowDir 6
#define shoulderDir 7

#define elbowPos 0
#define shoulderPos 1

#define rightRearPWM 6
#define rightRearIN1 10
#define rightRearIN2 11

#define leftRearPWM 9
#define leftRearIN1 12
#define leftRearIN2 13

#define rightFrontPWM 5
#define rightFrontIN1 8
#define rightFrontIN2 9

#define leftFrontPWM 10
#define leftFrontIN1 15
#define leftFrontIN2 14

#define leftRearPos 5
#define rightRearPos 7
#define leftFrontPos 6
#define rightFrontPos 4
