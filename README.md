# MSE-2202-DESIGN-CODE
This code was developed by John Van Heurn, Stephan Dhanraj, Eric Paglia and Anthony Guolla for use in our final design project for MSE 2202. Using an Arduino Platform , the robot is intended to pick up a cube, find a pyramid that emits an IR signal and place the cube under the pyramid. 
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

// FOR LEDS NUMBER 1 THROUGH 12
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
int LED = -2;
int stepComplete = 0;

// BEGIN SWITCH
int DPDTswitch = 3;
int timerCounter = 0;
int timer = 0;
int counter = 0;

// MOTORS
Servo servo_RightMotor;
Servo servo_LeftMotor;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// SENSORS 
const int hallEffect = A5;
const int IRsensor = 0;

// CLAW
Servo servo_Claw;
const int claw = 10;


void setup() {
  Wire.begin(); // required for 12CEncoder library
  Serial.begin(2400);
  // CHARLIEPLEXING SETUP
  CharliePlexM::set(ci_Charlieplex_LED1, ci_Charlieplex_LED2,ci_Charlieplex_LED3,ci_Charlieplex_LED4);
  // DRIVE MOTORS SETUP
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  // DRIVE ENCODERS SETUP
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  // BEGIN SWITCH SETUP
  pinMode(DPDTswitch, INPUT);
  // CLAW SETUP
  pinMode(claw,OUTPUT);
  servo_Claw.attach(claw);
  // SENSORS
  pinMode(hallEffect, INPUT);
  pinMode(IRsensor, INPUT);
}

void loop() {
  
  // CHARLIEPLEX SYNCS UP WITH STEPS
  CharliePlexM::Write(LED, 1);
  if (stepComplete == 1) {
    CharliePlexM::Write(LED, 0); // TO HAVE THE LEDS SWITCH HORIZONTALLY AFTER EACH STEP
    LED = LED + 3;
    if (LED >= 13) {
      LED = LED - 14;
      if (LED < 1) {
        LED = LED + 3;
      }
    }
    stepComplete = 0; // RESET STEP COMPLETE CHECK
    timerCounter = 0; // RESET TIMERS
    timer = 0;
    counter++; // NEXT STEP
  }

/*******************************************/
if (counter == 0) {
  stepComplete = 1;
  servo_Claw.write(0);
}
if (counter == 1 && digitalRead(DPDTswitch) == LOW) {
  if (timerCounter == 0) {
    timerCounter++;
    timer = millis();
  }
  if (millis() - timer >= 3000) { // INITIAL SWITCH
    stepComplete = 1;
  }
}
if (counter == 2) { // DRIVE ALONG THE WALL WITH CLAW OPEN
  servo_Claw.write(125);
  servo_LeftMotor.writeMicroseconds(1800); 
  servo_RightMotor.writeMicroseconds(1800);
  Serial.println(analogRead(hallEffect));
  if (analogRead(hallEffect) >= 525 || analogRead(hallEffect) <= 495) {
    stepComplete = 1;
    servo_LeftMotor.writeMicroseconds(0);
    servo_RightMotor.writeMicroseconds(0);
   }
}
if (counter == 3) { // CLAW CLOSE
  servo_Claw.write(0);
  // (turn it, for now testing IR)
  Serial.print(" IR Sensor: "); Serial.available(); Serial.println(Serial.read());
}
/*******************************************/

}
