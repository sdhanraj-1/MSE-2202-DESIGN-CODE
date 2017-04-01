#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

// CONSTANTS
int A = 65;
int E = 69;
int I = 73;
int O = 79;
int attempt = 0;

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
double timer = 0;
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

// CLAW AND PLATFORM
Servo servo_Claw;
const int claw = 10;
Servo servo_Platform;
const int platform = 11;

void setup() {
  
  Wire.begin(); // required for 12CEncoder library
  Serial.begin(2400); // please keep at 2400
  
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
  
  // CLAW SETUP AND PLATFORM  
  pinMode(claw,OUTPUT);
  servo_Claw.attach(claw);
  pinMode(platform,OUTPUT);
  servo_Platform.attach(platform);
  
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

if (counter > 0 && LED == -2) { // MAKES IT SO IF WE DECIDE TO TEST PARTWAY THROUGH OUR CODE BY SETTING COUNTER = X,
  LED = 1;                      // THE CHARLIEPLEX WILL ALWAYS START AT THE 1ST LED
}
if (counter == 0) { // INITIAL STEP TO GET THINGS SET UP, DOES NOT COUNT TOWARDS CHARLIEPLEX IF START FROM COUNTER == 0
  stepComplete = 1;
  servo_Claw.write(0);                                                                            // TEST THESE 2 NUMBERS
  servo_Platform.write(135);
}

if (counter == 1 && digitalRead(DPDTswitch) == LOW) {
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;
  }
  if (millis() - timer >= 3000) { // INITIAL SWITCH
    stepComplete = 1;
  }
}

if (counter == 2) { // DRIVE ALONG THE WALL WITH CLAW OPEN                                         // IMPLEMENT ULTRASONIC SENSOR TO BE WITHIN A CERTAIN DISTANCE OF WALL.
  servo_Claw.write(150);                                                                           // TEST THESE 2 NUMBERS
  servo_Platform.write(90);
  servo_LeftMotor.writeMicroseconds(1650);                                                         // TEST SPEEDS
  servo_RightMotor.writeMicroseconds(1650);
  Serial.println(analogRead(hallEffect));
  if (analogRead(hallEffect) >= 525 || analogRead(hallEffect) <= 495) {   // TEST SEE IF IT STOPS IN CORRECT PLACE
    stepComplete = 1;
    servo_LeftMotor.writeMicroseconds(0);
    servo_RightMotor.writeMicroseconds(0);
   }
}

if (counter == 3) { // CLAW CLOSE, WAIT 1 SECOND AND TURN PLATFORM
  if (timerCounter == 0) {
    timer = millis();
    timerCounter == 1;
  }
  servo_Claw.write(0);                                                                             // TEST THESE 2 NUMBERS 
  if (millis() - timer >= 1000) { // AFTER 1 SECOND, TURN PLATFORM, PROCEED TO NEXT STEP
    servo_Platform.write(0);
    stepComplete = 1;
  }
}

if (counter == 4) { // MOVE AWAY FROM WALL BRIEFLY BEFORE INITIATE FINDING PYRAMID
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;    
  }
  if (millis() - timer <= 1500) { // TURN RIGHT X SECONDS, IDEALLY 90 DEGREE TURN                                          // TEST THIS MOVEMENT
    servo_LeftMotor.writeMicroseconds(1650);
    servo_RightMotor.writeMicroseconds(1350);
  }
  if (millis() - timer > 1500 && millis() - timer < 4500) { // FORWARD TO GET AWAY FROM WALL
    servo_LeftMotor.writeMicroseconds(1650);
    servo_RightMotor.writeMicroseconds(1650);
  }
  if (millis() - timer >= 4500) {
    stepComplete = 1;
  }
}

if (counter == 5) { // (2ND LINE OF CHARLIEPLEX) INITIATE FINDING PYRAMID
                    // FOR LETTERS A AND E, MAY WANT TO IMPLEMENT SWITCH TO HAVE IT LOOK FOR O AND I INSTEAD LATER
  Serial.print(" IR Sensor: "); Serial.available(); Serial.println(Serial.read());
  if (Serial.read() == A || Serial.read() == E) {  // FORWARDS WHEN IT SEES PYRAMID
    servo_LeftMotor.writeMicroseconds(1650);
    servo_RightMotor.writeMicroseconds(1650); 
    attempt = 0;
    timerCounter = 0; 
  }
  if (Serial.read() != A && Serial.read() != E) { // SCAN LEFT AND RIGHT, RIGHT 1 SECOND, LEFT 2 SECONDS, RIGHT 4 SECONDS ETC....
    if (timerCounter = 0) {
      timer = millis();
      timerCounter = 1;
    }    
    if (attempt % 2 == 1) {
      servo_LeftMotor.writeMicroseconds(1650);
      servo_RightMotor.writeMicroseconds(1350);
      if ((millis() - timer) / 1000 >= pow(2,attempt)) {
        attempt++;
        timerCounter = 0;
      }
    }
    if (attempt %2 == 0) {
      servo_LeftMotor.writeMicroseconds(1350);
      servo_RightMotor.writeMicroseconds(1650);
      if ((millis() - timer) / 1000 >= pow(2,attempt)) {
        attempt++;
        timerCounter = 0;
      }
    }  
  }
  /*
  if (step condition met) {
    stepComplete = 1;
  } 
  */
}
/*******************************************/

}
