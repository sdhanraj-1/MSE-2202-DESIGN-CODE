#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

/*PRINT CODE LINES: 
  Serial.print(" Hall Effect Sensor: "); Serial.println(analogRead(hallEffect));
  Serial.print( " Tilt Sensor: "); Serial.println(digitalRead(tilt));
  Serial.print(" IR Sensor: "); Serial.available(); Serial.println(Serial.read());  
  Serial.print(" IR Distance Sensor: "); Serial.println(analogRead(IRdistance));
*/

// CONSTANTS
int A = 65;
int E = 69;
int I = 73;
int O = 79;
int clawOpen = 80;
int clawClosed = 15;
int platformOne = 150;
int platformTwo = 110;
int platformThree = 0;
int armOne = 50;        //  TEST 
int armTwo = 50;        //  TEST
int slowforward = 1635;
int forward = 1675;
int reverse = 1325;
int stationary = 0;
const long ultraUpper = 225;
const long ultraLower = 180;
const long distanceUpper = 300; // TEST
const long distanceLower = 200; // TEST

// FOR LEDS NUMBER 1 THROUGH 12
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;

// BEGIN
int timerCounter = 0;
int timer = 0;
int counter = 0;
int LED = -2;
int stepComplete = 0;
int attempt = 0;

// MOTORS
Servo servo_RightMotor;
Servo servo_LeftMotor;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;

// CLAW AND PLATFORM
Servo servo_Claw;
Servo servo_Platform;
const int claw = 10;
const int platform = 11;

// TIP MECHANISM
Servo servo_TipMotor;
Servo servo_Arm;
const int ci_TipMotor = 12;
const int arm = 13;

// SENSORS 
const int hallEffect = A5;
const int IRdistance = A4;
const int IRsensor = 0;
const int ci_Ultrasonic_Ping = 2;
const int ci_Ultrasonic_Data = 3;
unsigned long ul_Echo_Time;

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
  
  // CLAW SETUP AND PLATFORM  
  pinMode(claw,OUTPUT);
  servo_Claw.attach(claw);
  pinMode(platform,OUTPUT);
  servo_Platform.attach(platform);

  // TIP MECHANISM SETUP
  pinMode(arm, OUTPUT);
  servo_Arm.attach(arm);
  pinMode(ci_TipMotor,OUTPUT);
  servo_TipMotor.attach(ci_TipMotor);
  
  // SENSORS
  pinMode(hallEffect, INPUT);
  pinMode(IRsensor, INPUT);
  pinMode(IRdistance, INPUT);
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);
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
Ping();
if (counter > 0 && LED == -2) { // MAKES IT SO IF WE DECIDE TO TEST PARTWAY THROUGH OUR CODE BY SETTING COUNTER = X,
  LED = 1;                      // THE CHARLIEPLEX WILL ALWAYS START AT THE 1ST LED
}

if (counter == 0) { // INITIAL STEP TO GET THINGS SET UP, DOES NOT COUNT TOWARDS CHARLIEPLEX IF START FROM COUNTER = 0
  stepComplete = 1;  
  servo_Claw.write(clawClosed);                                                                
  servo_Platform.write(platformOne);
  servo_Arm.write(armOne);
}

if (counter == 1) {
  if (ul_Echo_Time > ultraLower && ul_Echo_Time < ultraUpper) {
    CharliePlexM::Write(12,1);
    if (timerCounter == 0) {
      timer = millis();
      timerCounter = 1;
    }
  }
  if (millis() - timer >= 2000 && ul_Echo_Time > ultraLower && ul_Echo_Time < ultraUpper) { 
    stepComplete = 1;
    CharliePlexM::Write(12,0);
  }
}

if (counter == 2) { // DRIVE ALONG THE WALL WITH CLAW OPEN              
  servo_Claw.write(clawOpen);                                                                  
  servo_Platform.write(platformTwo);
  if (ul_Echo_Time < ultraLower) {
    servo_RightMotor.writeMicroseconds(slowforward);
    servo_LeftMotor.writeMicroseconds(forward);
  }
  else if (ul_Echo_Time > ultraUpper) {
    servo_RightMotor.writeMicroseconds(forward);
    servo_LeftMotor.writeMicroseconds(slowforward);
  }
  else {
    servo_LeftMotor.writeMicroseconds(forward);                                             
    servo_RightMotor.writeMicroseconds(forward);
  }
  if (analogRead(hallEffect) >= 525 || analogRead(hallEffect) <= 495) {   
    stepComplete = 1;
    servo_LeftMotor.writeMicroseconds(stationary);
    servo_RightMotor.writeMicroseconds(stationary);
   }
}

if (counter == 3) { // CLAW CLOSE, WAIT 1 SECOND AND TURN PLATFORM
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;
  }
  servo_Claw.write(clawClosed);                                                                       
  if (millis() - timer >= 1000) { 
    servo_Platform.write(platformThree);
    stepComplete = 1;
  }
}

if (counter == 4) { // MOVE AWAY FROM WALL BRIEFLY BEFORE INITIATE FINDING PYRAMID
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;    
  }
  if (millis() - timer <= 2500) { // TURN RIGHT X SECONDS, IDEALLY 90 DEGREE TURN                                      
    servo_LeftMotor.writeMicroseconds(forward);
    servo_RightMotor.writeMicroseconds(reverse);
  }
  if (millis() - timer > 2500 && millis() - timer < 3200) { // FORWARD TO GET AWAY FROM WALL
    servo_LeftMotor.writeMicroseconds(forward);
    servo_RightMotor.writeMicroseconds(forward);
  }
  if (millis() - timer >= 3200) {
    stepComplete = 1;
  }
}

if (counter == 5) { // (2ND LINE OF CHARLIEPLEX) INITIATE FINDING PYRAMID
  if (Serial.read() == A || Serial.read() == E) {  // FORWARDS WHEN IT SEES PYRAMID
    servo_LeftMotor.writeMicroseconds(forward);
    servo_RightMotor.writeMicroseconds(forward); 
    attempt = 0;
    timerCounter = 0; 
  }
  else { // SCAN LEFT AND RIGHT, RIGHT 2 SECOND, LEFT 4 SECONDS, RIGHT 6 SECONDS ETC....
    if (timerCounter == 0) {
      timer = millis();
      timerCounter = 1;
    }    
    if (attempt % 2 == 1) {
      servo_LeftMotor.writeMicroseconds(forward);
      servo_RightMotor.writeMicroseconds(reverse);
      if (millis() - timer >= attempt*1000) {
        attempt++;
        timerCounter = 0;
      }
    }
    if (attempt %2 == 0) {
      servo_LeftMotor.writeMicroseconds(reverse);
      servo_RightMotor.writeMicroseconds(forward);
      if (millis() - timer >= attempt*1000) {
        attempt++;
        timerCounter = 0;
      }
    }  
  }
  if (analogRead(IRdistance) <= distanceUpper && analogRead(IRdistance) >= distanceLower) {
    stepComplete = 1;
  } 
}

if (counter == 6) { // PYRAMID WITHIN U OF FRAME, ADJUST PYRAMID QUICKLY
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;
  }
  servo_RightMotor.writeMicroseconds(forward);
  servo_LeftMotor.writeMicroseconds(slowforward);
  if (millis() - timer >= 600) {
    servo_RightMotor.writeMicroseconds(stationary);
    servo_LeftMotor.writeMicroseconds(stationary);
    stepComplete = 1;
  }
}

if (counter == 7) { // CLOSE ARM, SPIN MOTOR
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;
  }
  servo_Arm.write(armTwo);
  if (millis() - timer >= 1500 && millis() - timer <= 4000) {
    servo_TipMotor.writeMicroseconds(slowforward);
  }
  if (millis() - timer >= 4000) {
    stepComplete = 1;
  }
}

if (counter == 8) {
  if (timerCounter == 0) {
    timer = millis();
    timerCounter = 1;
  }
  servo_Claw.write(clawOpen);
  servo_Arm.write(armOne);
  if (millis() - timer >= 1000) {
    stepComplete = 1;
  }
}

if (counter == 9) { // DRIVE BACKWARDS ( LAST STEP ) , 3RD LINE OF CHARLIEPLEX
  if (timerCounter = 0) {
    timer = millis();
    timerCounter = 1;
  }   
  servo_RightMotor.writeMicroseconds(reverse);
  servo_LeftMotor.writeMicroseconds(reverse);
  if (millis() - timer >= 1500) {
    servo_RightMotor.writeMicroseconds(stationary);
    servo_LeftMotor.writeMicroseconds(stationary);
  }
}
/*******************************************/
}

void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);
  Serial.print( " Echo_Time: " ); Serial.println(ul_Echo_Time); 
}

