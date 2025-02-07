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
const float GEAR_RATIO_1 = 12.45;    // Physical gear ratio is 12.25 but this is calibrated
const float GEAR_RATIO_2 = 5.7;    // Example ratio for arm motor
const int GEAR_RATIO_3 = 3;    // Example ratio for Z-axis linear
const int GEAR_RATIO_4 = 3;    // Example ratio for Z-axis rotation

const float SPEED_MULTIPLIER = 1.5;

// Motor 1 Configuration (XY Base)
const int MICRO_STEPS_1 = 16;  
const int STEPS_PER_REV_1 = BASE_STEPS_PER_REV * MICRO_STEPS_1 * GEAR_RATIO_1;
const float MAX_SPEED_1 = 8000*SPEED_MULTIPLIER;
const float RUNNING_SPEED_1 = 7000*SPEED_MULTIPLIER;
const float ACCELERATION_1 = 8000*SPEED_MULTIPLIER;

// Motor 2 Configuration (XY Arm)
const int MICRO_STEPS_2 = 16;  
const int STEPS_PER_REV_2 = BASE_STEPS_PER_REV * MICRO_STEPS_2 * GEAR_RATIO_2;
const float MAX_SPEED_2 = 4000*SPEED_MULTIPLIER;
const float RUNNING_SPEED_2 = 3500*SPEED_MULTIPLIER;
const float ACCELERATION_2 = 4000*SPEED_MULTIPLIER;

// Motor 3 Configuration (Z Linear)
const int MICRO_STEPS_3 = 16;  
const int STEPS_PER_REV_3 = BASE_STEPS_PER_REV * MICRO_STEPS_3 * GEAR_RATIO_3;
const float MAX_SPEED_3 = 30000;
const float RUNNING_SPEED_3 = 26250;
const float ACCELERATION_3 = 30000;
const float SCREW_PITCH = 15; //(mm per revolution)

// Motor 4 Configuration (Z Rotation)
const int MICRO_STEPS_4 = 16;
const int STEPS_PER_REV_4 = BASE_STEPS_PER_REV * MICRO_STEPS_4 * GEAR_RATIO_4;
const float MAX_SPEED_4 = 30000;
const float RUNNING_SPEED_4 = 26250;
const float ACCELERATION_4 = 30000;

const float HOME_BACKOFF_DEGREES = 5;      // Degrees to back off after first contact
const float HOME_SLOW_SPEED = 1000;         // Slower homing speed for final approach
const float M1_HOME_OFFSET = 112.9;     // Final position offset from home
const float M2_HOME_OFFSET = 159;

// Safety limits for angles
const float THETA1_MIN = -110;  // Adjust these limits based on your robot's
const float THETA1_MAX = 110;   // mechanical constraints
const float THETA2_MIN = -155;     // Typically elbow angle has different
const float THETA2_MAX = 155;   // constraints than shoulder

// At the top with other constants
const float testAngles[2][2] = {
    {-98.895795, 107.791591},  // First position
    {-66.882045, 71.836577}   // Second position
};

bool isAngleSafe(float theta1, float theta2) {
    return (theta1 >= THETA1_MIN && theta1 <= THETA1_MAX &&
            theta2 >= THETA2_MIN && theta2 <= THETA2_MAX);
}

// Add these global variables at the top with your other declarations
float longestD2G;
char longestMotor;
float motor1D2G, motor2D2G, motor3D2G, motor4D2G;
float motor1StepDelay, motor2StepDelay, motor3StepDelay, motor4StepDelay;
float stepCounter;
float motor1StepCounter, motor2StepCounter, motor3StepCounter, motor4StepCounter;
float lastD2G;
int motor1Dir, motor2Dir, motor3Dir, motor4Dir;
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
    {150.000000, -150.000000, 0.000000, 0.000000, -8.895795, 107.791591, -98.895795, 0.0, 0.000000, 0.0},
    {-150.000000, 250.000000, 0.000000, 0.000000, -85.045468, -71.836577, 156.882045, 0.0, 0.000000, 0.0},
    {-150.000000, -250.000000, 0.000000, 0.000000, 85.045468, 71.836577, -156.882045, 0.0, 0.000000, 0.0},
    {150.000000, 150.000000, 0.000000, 0.000000, -98.895795, 107.791591, -8.895795, 0.0, 0.000000, 0.0}
};
// Format: {theta1, theta2, z_height, rotation, theta3}
const float angleWaypoints[][5] = {
    {-98.895795, 107.791591, 0.0, 0.0, -8.895795},
    {-8.895795, 107.791591, 50, 0.0, -98.895795},
    {-85.045468, -71.836577, 0.0, 0.0, 156.882045},
    {85.045468, 71.836577, 50, 0.0, -156.882045},
    {-98.895795, 107.791591, 0.0, 0.0, -8.895795}
};

// Global variables to track movement and waypoints
bool isHomed = false;
bool staticDebug = true; //set to false if you want it to home and nothing else
bool movementComplete = false;
int currentWaypoint = 0;
const int TOTAL_WAYPOINTS = 5;  // Total number of waypoints in the array

// Add this function after constants but before setup()
int32_t degreesToSteps(float degrees, int stepsPerRev) {
    return static_cast<int32_t>((degrees / 360.0) * stepsPerRev);
}

struct EndEffectorAngles {
    float motor3_angle;
    float motor4_angle;
};

EndEffectorAngles endEffectorMovement(float target_height_mm, float target_rotation_deg) {
    EndEffectorAngles angles;
    
    // For pure height change (no rotation):
    // Motor 3 rotates X degrees, Motor 4 stays still
    // 3 motor revolutions (1080°) = 15mm height change
    float height_motor4_angle = (target_height_mm / SCREW_PITCH)*360;
    
    // For pure rotation (no height change):
    // Both motors need to rotate the same amount
    // 3 motor revolutions (1080°) = 360° output rotation
    float rotation_angle = (target_rotation_deg / 360)*360;
    
    // Combine the movements
    angles.motor4_angle = -(height_motor4_angle + rotation_angle);
    angles.motor3_angle = -rotation_angle;
    
    return angles;
}

void coordinatedMotor1Move() {
    if (motor1StepDelay > 0) {
        if (motor1StepCounter < stepCounter) {
            digitalWrite(STEP_PIN_1, HIGH);
            delayMicroseconds(minPulseWidth);
            digitalWrite(STEP_PIN_1, LOW);

            MOTOR1.setCurrentPosition(MOTOR1.currentPosition() + motor1Dir);
            motor1StepCounter += motor1StepDelay;
        }
    }
}

void coordinatedMotor2Move() {
    if (motor2StepDelay > 0) {
        if (motor2StepCounter < stepCounter) {
            digitalWrite(STEP_PIN_2, HIGH);
            delayMicroseconds(minPulseWidth);
            digitalWrite(STEP_PIN_2, LOW);

            MOTOR2.setCurrentPosition(MOTOR2.currentPosition() + motor2Dir);
            motor2StepCounter += motor2StepDelay;
        }
    }
}

void coordinatedMotor3Move() {
    if (motor3StepDelay > 0) {
        if (motor3StepCounter < stepCounter) {
            digitalWrite(STEP_PIN_3, HIGH);
            delayMicroseconds(minPulseWidth);
            digitalWrite(STEP_PIN_3, LOW);

            MOTOR3.setCurrentPosition(MOTOR3.currentPosition() + motor3Dir);
            motor3StepCounter += motor3StepDelay;
        }
    }
}

void coordinatedMotor4Move() {
    if (motor4StepDelay > 0) {
        if (motor4StepCounter < stepCounter) {
            digitalWrite(STEP_PIN_4, HIGH);
            delayMicroseconds(minPulseWidth);
            digitalWrite(STEP_PIN_4, LOW);

            MOTOR4.setCurrentPosition(MOTOR4.currentPosition() + motor4Dir);
            motor4StepCounter += motor4StepDelay;
        }
    }
}

void moveToAngles(float theta1Target, float theta2Target, float theta3Target, float theta4Target) {
    static int stage = 0;

    if (stage == 0) {
        // Safety check
        if (!isAngleSafe(theta1Target, theta2Target)) {
            Serial.println("Warning: Target angles out of safe range!");
            stage = 0;
            movementComplete = true;
            return;
        }

        // Step 1: Calculate who has the farthest distance to travel
        MOTOR1.setMaxSpeed(MAX_SPEED_1);
        MOTOR2.setMaxSpeed(MAX_SPEED_2);
        MOTOR3.setMaxSpeed(MAX_SPEED_3);
        MOTOR4.setMaxSpeed(MAX_SPEED_4);
        
        // Convert target angles to steps
        int32_t motor1TargetSteps = degreesToSteps(theta1Target, STEPS_PER_REV_1);
        int32_t motor2TargetSteps = degreesToSteps(theta2Target, STEPS_PER_REV_2);
        int32_t motor3TargetSteps = degreesToSteps(theta3Target, STEPS_PER_REV_3);
        int32_t motor4TargetSteps = degreesToSteps(theta4Target, STEPS_PER_REV_4);
        
        motor1D2G = abs(motor1TargetSteps - MOTOR1.currentPosition());
        motor2D2G = abs(motor2TargetSteps - MOTOR2.currentPosition());
        motor3D2G = abs(motor3TargetSteps - MOTOR3.currentPosition());
        motor4D2G = abs(motor4TargetSteps - MOTOR4.currentPosition());
        
        longestD2G = 0;
        
        if (motor1D2G > longestD2G) {
            longestD2G = motor1D2G;
            longestMotor = '1';
        }
        if (motor2D2G > longestD2G) {
            longestD2G = motor2D2G;
            longestMotor = '2';
        }
        if (motor3D2G > longestD2G) {
            longestD2G = motor3D2G;
            longestMotor = '3';
        }
        if (motor4D2G > longestD2G) {
            longestD2G = motor4D2G;
            longestMotor = '4';
        }

        // Step 2: Calculate step delays for synchronized movement
        motor1StepDelay = (motor1D2G > 0) ? longestD2G / motor1D2G : 0;
        motor2StepDelay = (motor2D2G > 0) ? longestD2G / motor2D2G : 0;
        motor3StepDelay = (motor3D2G > 0) ? longestD2G / motor3D2G : 0;
        motor4StepDelay = (motor4D2G > 0) ? longestD2G / motor4D2G : 0;

        // Step 3: Initialize step counters
        stepCounter = 0;
        motor1StepCounter = motor1StepDelay - 1;
        motor2StepCounter = motor2StepDelay - 1;
        motor3StepCounter = motor3StepDelay - 1;
        motor4StepCounter = motor4StepDelay - 1;

        // Step 4: Set target positions based on longest moving motor
        switch(longestMotor) {
            case '1':
                MOTOR1.moveTo(motor1TargetSteps);
                lastD2G = MOTOR1.distanceToGo();
                break;
            case '2':
                MOTOR2.moveTo(motor2TargetSteps);
                lastD2G = MOTOR2.distanceToGo();
                break;
            case '3':
                MOTOR3.moveTo(motor3TargetSteps);
                lastD2G = MOTOR3.distanceToGo();
                break;
            case '4':
                MOTOR4.moveTo(motor4TargetSteps);
                lastD2G = MOTOR4.distanceToGo();
                break;
        }
        
        // Set target positions for all motors
        MOTOR1.moveTo(motor1TargetSteps);
        MOTOR2.moveTo(motor2TargetSteps);
        MOTOR3.moveTo(motor3TargetSteps);
        MOTOR4.moveTo(motor4TargetSteps);

        // Step 5: Set directions for all motors
        motor1Dir = (MOTOR1.distanceToGo() < 0) ? -1 : 1;
        motor2Dir = (MOTOR2.distanceToGo() < 0) ? -1 : 1;
        motor3Dir = (MOTOR3.distanceToGo() < 0) ? -1 : 1;
        motor4Dir = (MOTOR4.distanceToGo() < 0) ? -1 : 1;

        digitalWrite(DIR_PIN_1, (motor1Dir < 0) ? LOW : HIGH);
        digitalWrite(DIR_PIN_2, (motor2Dir < 0) ? LOW : HIGH);
        digitalWrite(DIR_PIN_3, (motor3Dir < 0) ? LOW : HIGH);
        digitalWrite(DIR_PIN_4, (motor4Dir < 0) ? LOW : HIGH);

        stage = 1;
    }

    if (stage == 1) {
        // Run motors with synchronized movement
        if (lastD2G != 0) {  // Only update step counter if we're still moving
            switch(longestMotor) {
                case '1':
                    if (lastD2G != MOTOR1.distanceToGo()) {
                        stepCounter += motor1StepDelay;
                        lastD2G = MOTOR1.distanceToGo();
                    }
                    MOTOR1.run();
                    break;
                case '2':
                    if (lastD2G != MOTOR2.distanceToGo()) {
                        stepCounter += motor2StepDelay;
                        lastD2G = MOTOR2.distanceToGo();
                    }
                    MOTOR2.run();
                    break;
                case '3':
                    if (lastD2G != MOTOR3.distanceToGo()) {
                        stepCounter += motor3StepDelay;
                        lastD2G = MOTOR3.distanceToGo();
                    }
                    MOTOR3.run();
                    break;
                case '4':
                    if (lastD2G != MOTOR4.distanceToGo()) {
                        stepCounter += motor4StepDelay;
                        lastD2G = MOTOR4.distanceToGo();
                    }
                    MOTOR4.run();
                    break;
            }

            // Run coordinated moves for all other motors
            if (longestMotor != '1') coordinatedMotor1Move();
            if (longestMotor != '2') coordinatedMotor2Move();
            if (longestMotor != '3') coordinatedMotor3Move();
            if (longestMotor != '4') coordinatedMotor4Move();
        }

        // Check if movement is complete
        if (MOTOR1.distanceToGo() == 0 && MOTOR2.distanceToGo() == 0 && 
            MOTOR3.distanceToGo() == 0 && MOTOR4.distanceToGo() == 0) {
            stage = 0;
            movementComplete = true;
            Serial.println("Synchronized movement completed");
        }
    }
}

void home() {
   // First ensure MOTOR2 is disabled during MOTOR1's initial movement
   digitalWrite(ENABLE_PIN_2, HIGH);  // Disable MOTOR2 (HIGH = disabled)
   digitalWrite(ENABLE_PIN_4, HIGH);


   MOTOR3.setSpeed(HOME_SLOW_SPEED);
   MOTOR3.setAcceleration(0);
   MOTOR3.move(degreesToSteps(-2000, STEPS_PER_REV_3));
   while (MOTOR3.distanceToGo() != 0) {
       MOTOR3.run();
   }
   MOTOR3.move(degreesToSteps(90, STEPS_PER_REV_3));
   while (MOTOR3.distanceToGo() != 0) {
       MOTOR3.run();
   }
   MOTOR3.stop();
   delay(1000);
   digitalWrite(ENABLE_PIN_4, LOW);

//    EndEffectorAngles targetAngles = endEffectorMovement(15, 360);

//    MOTOR4.setSpeed(HOME_SLOW_SPEED);
//    MOTOR4.setAcceleration(0);
//    MOTOR4.move(degreesToSteps(targetAngles.motor4_angle, STEPS_PER_REV_4));
//    while (MOTOR4.distanceToGo() != 0) {
//        MOTOR4.run();
//    }
//    MOTOR4.stop();
//    delay(1000);

//    MOTOR3.setSpeed(HOME_SLOW_SPEED);
//    MOTOR3.setAcceleration(0);
//    MOTOR3.move(degreesToSteps(targetAngles.motor3_angle, STEPS_PER_REV_3));
//    while (MOTOR3.distanceToGo() != 0) {
//        MOTOR3.run();
//    }
//    MOTOR3.stop();
//    delay(1000);
   MOTOR1.setSpeed(500);
   MOTOR1.setAcceleration(0);
   
   // Original MOTOR2 pre-movement can be removed since motor is disabled
   
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

   // Now that MOTOR1 is at its limit switch, we can safely enable MOTOR2
   digitalWrite(ENABLE_PIN_2, LOW);  // Enable MOTOR2
   delay(100);  // Give a small delay for the enable to take effect

   // Now continue with MOTOR2 homing sequence as before
   MOTOR2.setSpeed(-HOME_SLOW_SPEED/2);
   while (digitalRead(LIMIT_SWITCH_2) == HIGH) {
       MOTOR2.runSpeed();
   }
   MOTOR2.stop();
   delay(100);

   // Back off slowly until switch releases, then go extra distance
   MOTOR2.setSpeed(HOME_SLOW_SPEED/2);
   while (digitalRead(LIMIT_SWITCH_2) == LOW) {  // Until switch releases
       MOTOR2.runSpeed();
   }
   // Extra backoff distance
   int32_t extraSteps2 = degreesToSteps(HOME_BACKOFF_DEGREES, STEPS_PER_REV_2);
   for(int32_t i = 0; i < extraSteps2; i++) {
       MOTOR2.runSpeed();
   }
   delay(100);
   
   MOTOR2.setSpeed(-HOME_SLOW_SPEED/2);
   while (digitalRead(LIMIT_SWITCH_2) == HIGH) {
       MOTOR2.runSpeed();
   }
   MOTOR2.stop();
   delay(100);
   
   MOTOR2.setSpeed(HOME_SLOW_SPEED/2);
   MOTOR2.move(degreesToSteps(-1, STEPS_PER_REV_2));
   while (MOTOR2.distanceToGo() != 0) {
       MOTOR2.run();
   }
   MOTOR2.stop();
   delay(100);

   MOTOR2.setSpeed(HOME_SLOW_SPEED/2);
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
   // After moving to the straight-out position, set this as our zero
   MOTOR1.setCurrentPosition(0);
   MOTOR2.setCurrentPosition(0);
   MOTOR3.setCurrentPosition(0);
   MOTOR4.setCurrentPosition(0);
   Serial.println("Homing complete - Position reset to 0,0");
   isHomed = true;
}

void setup() {
  // Configure limit switch pin as input with pullup
  pinMode(LIMIT_SWITCH_1, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
  
  // Configure motor pins
  pinMode(ENABLE_PIN_1, OUTPUT);
  digitalWrite(ENABLE_PIN_1, LOW); // Enable motor 1
  pinMode(ENABLE_PIN_2, OUTPUT);
  digitalWrite(ENABLE_PIN_2, LOW); // Enable motor 2
  pinMode(ENABLE_PIN_3, OUTPUT);
  digitalWrite(ENABLE_PIN_3, LOW); // Enable motor 3
  pinMode(ENABLE_PIN_4, OUTPUT);
  digitalWrite(ENABLE_PIN_4, LOW); // Enable motor 4
  
  // Configure motor parameters
  MOTOR1.setMaxSpeed(MAX_SPEED_1);
  MOTOR1.setAcceleration(ACCELERATION_1);
  MOTOR2.setMaxSpeed(MAX_SPEED_2);
  MOTOR2.setAcceleration(ACCELERATION_2);
  MOTOR3.setMaxSpeed(MAX_SPEED_3);
  MOTOR3.setAcceleration(ACCELERATION_3);
  MOTOR4.setMaxSpeed(MAX_SPEED_4);
  MOTOR4.setAcceleration(ACCELERATION_4);
  
  // Start serial for debugging (optional)
  Serial.begin(9600);
  
  // Wait 5 seconds before starting homing
  delay(5000);
  
  // Start homing sequence
  home();
}


void loop() {
    static bool initialMovementDone = false;
    static bool moveStarted = false;

    if(staticDebug) {
        if (isHomed && !initialMovementDone) {
            if (!moveStarted) {
                Serial.print("Moving to waypoint ");
                Serial.println(currentWaypoint);
                moveStarted = true;
            }
            
            // Convert z_height and rotation to motor angles using endEffectorMovement
            EndEffectorAngles endEffectorAngles = endEffectorMovement(
                angleWaypoints[currentWaypoint][2],  // z_height
                -(angleWaypoints[currentWaypoint][3]+angleWaypoints[currentWaypoint][4])   // rotation
            );
            
            moveToAngles(angleWaypoints[currentWaypoint][0],   // theta1
                        angleWaypoints[currentWaypoint][1],    // theta2
                        endEffectorAngles.motor3_angle,        // Convert z and rotation to motor3 angle
                        endEffectorAngles.motor4_angle);       // Convert z and rotation to motor4 angle
            
            if (movementComplete) {
                Serial.println("Waypoint reached");
                movementComplete = false;
                moveStarted = false;
                currentWaypoint++;
                
                if (currentWaypoint >= 5) {
                    currentWaypoint = 0;  // Reset to start
                    Serial.println("Completed full sequence");
                }
                delay(50);
            }
        }
    }    
}