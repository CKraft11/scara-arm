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
const int GEAR_RATIO_2 = 19/3;    // Example ratio for arm motor
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