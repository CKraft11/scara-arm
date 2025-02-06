#include <Arduino.h>
#include <AccelStepper.h>
// First Motor Pins (XY-Axis Base Motor)
const int STEP_PIN_1 = 11;    
const int DIR_PIN_1 = 13;    
const int ENABLE_PIN_1 = 7;  
const int LIMIT_SWITCH_1 = A5;

// Second Motor Pins (XY-Axis Arm Motor)
const int STEP_PIN_2= 10;    
const int DIR_PIN_2 = 12;    
const int ENABLE_PIN_2 = 8;  
const int LIMIT_SWITCH_2 = A4;

// Third Motor Pins (Z-Axis Linear Motion)
const int STEP_PIN_3 = 5;    
const int DIR_PIN_3 = 6;    
const int ENABLE_PIN_3 = 2;  

// Fourth Motor Pins (Z-Axis Rotation)
const int STEP_PIN_4 = 9;    
const int DIR_PIN_4 = 3;    
const int ENABLE_PIN_4 = 4;  

// Base configuration
const int BASE_STEPS_PER_REV = 200;  

// Gear Ratios for each motor
const int GEAR_RATIO_1 = 12.25;    // Example ratio for base motor
const int GEAR_RATIO_2 = 1;    // Example ratio for arm motor
const int GEAR_RATIO_3 = 3;    // Example ratio for Z-axis linear
const int GEAR_RATIO_4 = 3;    // Example ratio for Z-axis rotation

// Motor 1 Configuration (XY Base)
const int MICRO_STEPS_1 = 16;  
const int STEPS_PER_REV_1 = BASE_STEPS_PER_REV * MICRO_STEPS_1 * GEAR_RATIO_1;
const float MAX_SPEED_1 = 4000;
const float RUNNING_SPEED_1 = 3500;
const float ACCELERATION_1 = 4000;

// Motor 2 Configuration (XY Arm)
const int MICRO_STEPS_2 = 16;  
const int STEPS_PER_REV_2 = BASE_STEPS_PER_REV * MICRO_STEPS_2 * GEAR_RATIO_2;
const float MAX_SPEED_2 = 4000;
const float RUNNING_SPEED_2 = 3500;
const float ACCELERATION_2 = 4000;

// Motor 3 Configuration (Z Linear)
const int MICRO_STEPS_3 = 16;  
const int STEPS_PER_REV_3 = BASE_STEPS_PER_REV * MICRO_STEPS_3 * GEAR_RATIO_3;
const float MAX_SPEED_3 = 16000;
const float RUNNING_SPEED_3 = 14000;
const float ACCELERATION_3 = 16000;

// Motor 4 Configuration (Z Rotation)
const int MICRO_STEPS_4 = 16;
const int STEPS_PER_REV_4 = BASE_STEPS_PER_REV * MICRO_STEPS_4 * GEAR_RATIO_4;
const float MAX_SPEED_4 = 16000;
const float RUNNING_SPEED_4 = 14000;
const float ACCELERATION_4 = 16000;

const float HOME_BACKOFF_DEGREES = 5;      // Degrees to back off after first contact
const float HOME_SLOW_SPEED = 200.0;         // Slower homing speed for final approach
const float M1_HOME_OFFSET = 115;     // Final position offset from home
const float M2_HOME_OFFSET = 155;

// Add these global variables at the top with your other declarations
float longestD2G;
char longestMotor;
float motor1D2G, motor2D2G;
float motor1StepDelay, motor2StepDelay;
float stepCounter;
float motor1StepCounter, motor2StepCounter;
float lastD2G;
int motor1Dir, motor2Dir;
const int minPulseWidth = 20;

// Create stepper instances
AccelStepper MOTOR1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper MOTOR2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper MOTOR3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
AccelStepper MOTOR4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);

// waypoints[N][10] contains the following data for each waypoint N:
// [0] = X position (mm)
// [1] = Y position (mm)
// [2] = Z position (mm)
// [3] = Rotation (degrees)
// [4] = Theta1 (degrees)
// [5] = Theta2 (degrees)
// [6] = Theta3 (degrees)
// [7] = StopAtPoint (1.0 = stop, 0.0 = don't stop)
// [8] = Duration (seconds)
// [9] = LinearPath (1.0 = linear movement, 0.0 = curved movement)
const double waypoints[5][10] = {
    {150.000000, 150.000000, 0.000000, 0.000000, -98.895795, 107.791591, -8.895795, 0.0, 0.000000, 0.0},
    {250.000000, 150.000000, 0.000000, 0.000000, -66.882045, 71.836577, -4.954532, 0.0, 0.000000, 0.0},
    {250.000000, 50.000000, 0.000000, 0.000000, -56.221513, 89.823161, -33.601648, 0.0, 0.000000, 0.0},
    {150.000000, 50.000000, 0.000000, 0.000000, -82.381766, 127.893635, -45.511868, 0.0, 0.000000, 0.0},
    {150.000000, 150.000000, 0.000000, 0.000000, -98.895795, 107.791591, -8.895795, 0.0, 0.000000, 0.0}
};

// Global variable to track homing state
bool isHomed = false;

// Add this function after constants but before setup()
int32_t degreesToSteps(float degrees, int stepsPerRev) {
    return static_cast<int32_t>((degrees / 360.0) * stepsPerRev);
}

void home() {
   MOTOR1.setSpeed(500);
   MOTOR1.setAcceleration(0);
   MOTOR2.setSpeed(-250);
   MOTOR2.setAcceleration(0);

   MOTOR2.move(degreesToSteps(3, STEPS_PER_REV_1));
   while (MOTOR2.distanceToGo() != 0) {
       MOTOR2.run();
   }
   MOTOR2.stop();
   delay(1000);
   
   MOTOR1.setSpeed(HOME_SLOW_SPEED);

   MOTOR1.move(degreesToSteps(-3, STEPS_PER_REV_1));
   while (MOTOR1.distanceToGo() != 0) {
       MOTOR1.run();
   }
   MOTOR1.stop();
   delay(1000);
   
   MOTOR1.setSpeed(HOME_SLOW_SPEED);
   while (digitalRead(LIMIT_SWITCH_1) == HIGH) {
       MOTOR1.runSpeed();
   }
   MOTOR1.stop();
   delay(100);
   
   // Back off slowly until switch releases, then go extra distance
   MOTOR1.setSpeed(-HOME_SLOW_SPEED);
   while (digitalRead(LIMIT_SWITCH_1) == LOW) {  // Until switch releases
       MOTOR1.runSpeed();
   }
   // Extra backoff distance
   int32_t extraSteps1 = degreesToSteps(HOME_BACKOFF_DEGREES, STEPS_PER_REV_1);
   for(int32_t i = 0; i < extraSteps1; i++) {
       MOTOR1.runSpeed();
   }
   MOTOR1.stop();
   delay(100);

   MOTOR1.setSpeed(HOME_SLOW_SPEED);
   while (digitalRead(LIMIT_SWITCH_1) == HIGH) {
       MOTOR1.runSpeed();
   }
   MOTOR1.stop();
   delay(1000);
   
   MOTOR1.setSpeed(HOME_SLOW_SPEED);
   MOTOR1.move(degreesToSteps(1, STEPS_PER_REV_1));
   while (MOTOR1.distanceToGo() != 0) {
       MOTOR1.run();
   }
   MOTOR1.stop();
   delay(1000);

   MOTOR2.setSpeed(-HOME_SLOW_SPEED/10);
   while (digitalRead(LIMIT_SWITCH_2) == HIGH) {
       MOTOR2.runSpeed();
   }
   MOTOR2.stop();
   delay(100);

   // Back off slowly until switch releases, then go extra distance
   MOTOR2.setSpeed(HOME_SLOW_SPEED/10);
   while (digitalRead(LIMIT_SWITCH_2) == LOW) {  // Until switch releases
       MOTOR2.runSpeed();
   }
   // Extra backoff distance
   int32_t extraSteps2 = degreesToSteps(HOME_BACKOFF_DEGREES, STEPS_PER_REV_2);
   for(int32_t i = 0; i < extraSteps2; i++) {
       MOTOR2.runSpeed();
   }
   delay(100);
   
   MOTOR2.setSpeed(-HOME_SLOW_SPEED/10);
   while (digitalRead(LIMIT_SWITCH_2) == HIGH) {
       MOTOR2.runSpeed();
   }
   MOTOR2.stop();
   delay(100);
   
   MOTOR2.setSpeed(HOME_SLOW_SPEED/10);
   MOTOR2.move(degreesToSteps(-1, STEPS_PER_REV_2));
   while (MOTOR2.distanceToGo() != 0) {
       MOTOR2.run();
   }
   MOTOR2.stop();
   delay(100);

   MOTOR2.setSpeed(HOME_SLOW_SPEED/10);
   MOTOR2.move(degreesToSteps(M2_HOME_OFFSET, STEPS_PER_REV_2));
   while (MOTOR2.distanceToGo() != 0) {
       MOTOR2.run();
   }
   MOTOR2.stop();
   delay(100);
   
   MOTOR1.setSpeed(HOME_SLOW_SPEED);
   MOTOR1.move(degreesToSteps(-M1_HOME_OFFSET, STEPS_PER_REV_1));
   while (MOTOR1.distanceToGo() != 0) {
       MOTOR1.run();
   }
   MOTOR1.stop();
   delay(1000);
   
   isHomed = true;
}

void setup() {
  // Configure limit switch pin as input with pullup
  pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
  
  // Configure motor pins
  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW); // Enable the motor
  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW); // Enable the motor
  
  // Configure motor parameters
  MOTOR1.setMaxSpeed(MAX_SPEED_1);
  MOTOR1.setAcceleration(ACCELERATION_1);
  MOTOR2.setMaxSpeed(MAX_SPEED_2);
  MOTOR2.setAcceleration(ACCELERATION_2);
  
  // Start serial for debugging (optional)
  Serial.begin(9600);
  
  // Wait 5 seconds before starting homing
  delay(5000);
  
  // Start homing sequence
  home();
}


void loop() {
}