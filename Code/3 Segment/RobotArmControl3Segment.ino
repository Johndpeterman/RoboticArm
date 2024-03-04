#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <Ramp.h>
#include <math.h>
#include <Stepper.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SHOULDER_SERVOPIN 0
#define ELBOW_SERVOPIN 1
#define WRIST_SERVOPIN 2
#define GRIP_SERVOPIN 3

#define lowerLength 111    //mm 
#define upperLength 92

const int stepsPerRev = 2000;
const int rolePerMin = 15;
Stepper stepper(stepsPerRev, 8, 10, 9, 11);

const int MIN_MS = 500;
const int MAX_MS = 2500;

const int initValY = 30;
const int initValZ = 150;

double microValueShoulder;
double microValueElbow;
double microValueWrist;
double angle2;
double mappedAngle;
double angle3;
double angle4;

double L;
double Joint2; //shoulder joint
double Joint3; //elbow joint
double B;
double A;
double Joint2Length = lowerLength;
double Joint3Length = upperLength;


void calcIK(double y, double z) {

  L = sqrt((y*y) + (z*z));

  Joint3 = acos(  ( (Joint2Length * Joint2Length) + (Joint3Length * Joint3Length) - (L * L) ) / (2 * Joint2Length * Joint3Length) ) * (180 / PI);

  B = acos(  ( (L * L) + (Joint2Length * Joint2Length) - (Joint3Length * Joint3Length) ) / (2 * L * Joint2Length) ) * (180 / PI);

  A = atan(z / y) * (180 / PI);

  Joint2 = B + A;

  updatePos(Joint2, Joint3);

}



void updatePos(double angle2, double angle3) {

  //shoulder
  Serial.print("Shoulder: ");
  Serial.println(angle2);

  mappedAngle = map(angle2, 0, 180, 180, 0); //reverse direction
  microValueShoulder = map(mappedAngle, 0, 180, 500, 2500);
  pwm.writeMicroseconds(0, microValueShoulder);
  
  
  //elbow
  Serial.print("Elbow: ");
  Serial.println(angle3);
  Serial.println("////////////////////////////");

  microValueElbow = map(angle3, 0, 180, 500, 2500);
  pwm.writeMicroseconds(1, microValueElbow);

}


double angleSmoothed;

//set equal to an initial position that the servo will always be at when it starts
double anglePrev = 0;
double msVal;
boolean flag = true;

void goToAngle(double angleTarget) {

  flag = true;

  while (flag == true) {

    angleSmoothed = (angleTarget * .05) + (anglePrev * .95);
    anglePrev = angleSmoothed;

    Serial.print("Smoothed: ");
    Serial.println(angleSmoothed);
  
    msVal = map(angleSmoothed, 0, 180, 500, 2400);
    pwm.writeMicroseconds(WRIST_SERVOPIN, msVal);


    if ((angleSmoothed > angleTarget - .7) && (angleSmoothed < angleTarget + .7)) {
      flag = false;
    }

    delay(30);
  }

  anglePrev = angleTarget;
}


double yAct;
double zAct;

rampDouble yTar;
rampDouble zTar;

boolean atPosY1 = false;
boolean atPosZ1 = false;
boolean flag1 = true;

//set equal to an initial position that the arm will always be at when it starts
double prev_yTarget = initValY;
double prev_zTarget = initValZ;

void goToInterpolated(double yTarget, double zTarget) {

  yTar = prev_yTarget;
  zTar = prev_zTarget;

  atPosY1 = false;
  atPosZ1 = false;
  flag1 = true;

  yTar.go(yTarget, 1000); //start interpolation
  zTar.go(zTarget, 1000);
  
  while (flag1 == true) {

    yAct = yTar.update();
    zAct = zTar.update();

  /*
    Serial.print("yAct: ");
    Serial.println(yAct);
    Serial.print("zAct: ");
    Serial.println(zAct);
  */
    calcIK(yAct, zAct);
    
    if ((yAct > yTarget - .1) && (yAct < yTarget + .1)) {
      atPosY1 = true;
    }

    if ((zAct > zTarget - .1) && (zAct < zTarget + .1)) {
      atPosZ1 = true;
    }

    if (atPosY1 == true && atPosZ1 == true) {
      flag1 = false;
    }

    delay(30);
  }

  prev_yTarget = yTarget;
  prev_zTarget = zTarget;

}

const int closedGripMS = 850;
const int openGripMS = 1500;

void closeGrip() {
  pwm.writeMicroseconds(3, closedGripMS);
}
void openGrip() {
  pwm.writeMicroseconds(3, openGripMS);
}

void returnToDefault() {

  delay(3000);
  goToInterpolated(30, 150);
  goToAngle(0);
  //stepper.step(-1600);
  openGrip();

}

void setup() {
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  stepper.setSpeed(rolePerMin);

  Serial.begin(115200);

  /*
  angle2 = 90;
  mappedAngle = map(angle2, 0, 180, 180, 0);
  microValueShoulder = map(mappedAngle, 0, 180, 500, 2500);
  pwm.writeMicroseconds(0, microValueShoulder);

  angle3 = 90;
  microValueElbow = map(angle3, 0, 180, 500, 2500);
  pwm.writeMicroseconds(1, microValueElbow);
  
  
  angle4 = 90;
  microValueWrist = map(angle4, 0, 180, 500, 2400);
  pwm.writeMicroseconds(2, microValueWrist);

  calcIK(initValY, initValZ);
  openGrip();

  delay(3000);
  */
  

}

void drawSquare() {

  goToInterpolated(100, 50);
  delay(1500);
  goToInterpolated(150, 55);
  delay(1500);
  goToInterpolated(150, 100);
  delay(1500);
  goToInterpolated(100, 100);
  delay(1500);

}

void simplePickUp() {

  goToInterpolated(100, 150);
  goToAngle(20);
  closeGrip();
  delay(1000);
  

  goToInterpolated(30, 150);
  delay(500);
  stepper.step(600);
  delay(500);
  goToAngle(120);
  delay(1000);
  goToInterpolated(30, 100);
  delay(1000);
  goToInterpolated(100, 100);
  goToAngle(140);
  delay(1000);
  openGrip();



  returnToDefault();
  delay(10000000000000000);

}

void simpleHandOff() {

  goToAngle(140);
  goToInterpolated(30, 90);
  delay(1000);
  goToInterpolated(100, 90);
  delay(500);
  closeGrip();
  delay(1000);
  
  goToInterpolated(30, 90);
  delay(1000);
  stepper.step(800);
  delay(1000);
  goToInterpolated(100, 90);
  delay(500);
  goToInterpolated(100, 110);
  delay(1000);
  openGrip();
  delay(100);



  returnToDefault();
  delay(100000000);

}

void simpleTransport() {

  goToInterpolated(30, 170);
  goToAngle(40);
  delay(1500);
  goToInterpolated(55, 170);
  delay(500);
  closeGrip();
  delay(500);
  goToInterpolated(20, 170);
  delay(500);
  goToInterpolated(30, 140);
  delay(500);

  stepper.step(1600);
  delay(1000);

  goToInterpolated(55, 100);
  openGrip();
  delay(1000);

  goToInterpolated(55, 140);
  delay(1000);
  goToInterpolated(115, 100);
  goToAngle(55);
  delay(500);
  closeGrip();
  delay(500);
  goToInterpolated(115, 130);
  delay(500);
  goToInterpolated(30, 175);
  goToAngle(40);
  delay(500);
  goToInterpolated(55, 172);
  delay(500);
  stepper.step(-1600);
  delay(500);
  openGrip();

  returnToDefault();
  delay(20000);
}


void loop() {

  simpleTransport();

}
