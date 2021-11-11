// --------- Library Imports ---------                           // --------- Library Imports ---------

#include <Adafruit_MotorShield.h>                                // Import motor shield library

 

// --------- Variables ---------                                 // --------- Variables ---------

byte robot_test_state = 4;                                       // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details

 

long tick_counter = 0;                                           // Counts the number of ticks elapsed since program started running

const int tick_length = 5;                                       // The length of one tick in milliseconds. Clock Frequency = 1/tick_length

 

byte robot_state = 0;                                            // Variable to track the stage of the problem (0=Driving on line,1=Driving off line,2=Stopped)

byte no_dummies_rescued = 0;                                     // Number of dummies returned to their zones

 

byte left_motor_output, right_motor_output;                      // Voltage/Current/Whatever for controlling the speed/torque of each wheel motor. 1 = Front left, 2 = Front right, 3 = Back left, 4 = Back right

int line_detector_1, line_detector_2, line_detector_3;           // output of line detector (0-1023) 1 LHS 2 middle 3 RHS

const byte left_motor_port =  3;                                 // Port of the motor shield that the left motor uses

const byte right_motor_port = 4;                                 // Port of the motor shield that the right motor uses

 

// --------- Motor Initialisation ---------                      // --------- Motor Initialisation ---------

Adafruit_MotorShield AFMS = Adafruit_MotorShield();              // Create motorshield object called 'AFMS' with default I2C address. Put I2C address in brackets if different address needed

Adafruit_DCMotor *left_motor = AFMS.getMotor(left_motor_port);   // Initialise left  motor using assigned motor shield port

Adafruit_DCMotor *right_motor = AFMS.getMotor(right_motor_port); // Initialise right motor using assigned motor shield port

 

// --------- Functions ---------                                 // --------- Functions ---------

void drive_motor(Adafruit_DCMotor* motor, int spd, bool rev) {    // Electrical function that drives a single motor. motor: 0=Left,1=Right; spd: 0-255; rev: bool, true = Reverse

 

  if (rev) {

    motor->run(BACKWARD);

    }

  else {

    motor->run(FORWARD);

  }

  motor->setSpeed(spd);

 

  if (spd != 0) {

    if (tick_counter*tick_length % 500 < 250) {

      analogWrite(6,255);

    }

    else if (tick_counter*tick_length % 500 > 249) {

      analogWrite(6,0);

    }

  }

  else {

    analogWrite(6,0);

  }

 

  // ELECTRICAL FUNCTION HERE INCL. LED FLASH (USE GLOBAL 'tick_counter')

}
 
void follow_line() {                                             // Function that drives the motors and uses line sensors to move allow the line. Doesn't take inputs to stop (only call this function if the path is clear)
  if ((line_detector_2 > 500) and (line_detector_1 < 500) and (line_detector_3 < 500)){ //default on the line go straight ahead case
    drive_motor(left_motor_port, 250, false);
    drive_motor(right_motor_port, 250, false);
  }
  else if ((line_detector_2 < 500) and (line_detector_1 < 500) and (line_detector_3 < 500)){ //central detector off line but niether side on line yet but carry on straight
    drive_motor(left_motor_port, 200, false);
    drive_motor(right_motor_port, 200, false);
  }
  else if ((line_detector_2 < 500) and (line_detector_1 > 500) and (line_detector_3 < 500)){ //hit line on LHS so steering RIGHT
    drive_motor(left_motor_port, 250, false);
    drive_motor(right_motor_port, 200, false);
  }
  else if ((line_detector_2 < 500) and (line_detector_1 > 500) and (line_detector_3 < 500)){ //hit line on RHS so steering LEFT
    drive_motor(left_motor_port, 200, false);
    drive_motor(right_motor_port, 250, false);
  }
  else if ((line_detector_2 > 500) and (line_detector_1 > 500) and (line_detector_3 > 500)){ //hit horizontal line
    drive_motor(left_motor_port, 0, false);
    drive_motor(right_motor_port, 0, false);
  }
  else{ //some funky angles going on here, not an ideal case just sorta spin a'c'wise I guess
    drive_motor(right_motor_port, 100, false);
    drive_motor(left_motor_port, 100, true);
  }
}

void search_for_dummies(){
    drive_motor(right_motor_port, 100, false);                   // spin to the left
    drive_motor(left_motor_port, 100, true);
    //signal_strength = 0 // Continue here
}

// --------- Setup Function ---------                            // --------- Setup Function ---------

void setup() {                                                   // Function that runs on power-up/RESET

  Serial.begin(9600);                                            // Start Serial to print debug info

 

  while (!AFMS.begin()) {                                        // Wait until motorshield successfully initialised

    Serial.println("Failed to find motorshield.");

    delay(1000);                 

  }

  Serial.print("Motorshield successfully initialised.");

 

  robot_state = 0;                                               // Reset robot state to default

 

  left_motor->setSpeed(0);                                       // Initialise left motor

  left_motor->run(FORWARD);

  left_motor->run(RELEASE); 

 

  right_motor->setSpeed(0);                                       // Initialise right motor

  right_motor->run(FORWARD);

  right_motor->run(RELEASE); 

  

}

 

 

// --------- Main Loop ---------                                 // --------- Main Loop ---------

void loop() {                                                    // Function that runs repeatedly whilst the robot is on

  if (robot_test_state == 1) {                                   // Test 1: Spin the wheels without control of any kind

    if (tick_counter*tick_length > 3000) {                       // Wait 3 seconds before beginning test

      drive_motor(left_motor,255,false);                                    // Use the electrical function to spin each motor at max speed

      drive_motor(right_motor,255,false);

    }

  }

  else if (robot_test_state == 2 or robot_test_state == 3) {        // Test 2/3: Drive forwards for a fixed time (to give ~1m forwards)

    if (tick_counter*tick_length > 7000) {                       // Drive for [2] seconds

      drive_motor(left_motor,0,false);                                    // Stop motors after test complete

      drive_motor(right_motor,0,false);

    }

    else if (tick_counter*tick_length > 3000) {                  // Wait [3] seconds before beginning test

      drive_motor(left_motor,255,false);                                  // Drive both motors forwards at full speed

      drive_motor(right_motor,255,false);

    }

  }

  else if (robot_test_state == 4) {

    if (tick_counter*tick_length > 11000) {                       // Drive for [2] seconds

      drive_motor(left_motor,0,true);                                    // Stop motors after test complete

      drive_motor(right_motor,0,true);

    }

    else if (tick_counter*tick_length > 9000) {                       // Drive for [2] seconds

      drive_motor(left_motor,255,true);                                    // Stop motors after test complete

      drive_motor(right_motor,255,true);

    }

    else if (tick_counter*tick_length > 6000) {                  // Wait [3] seconds before beginning test

      drive_motor(left_motor,255-255*(tick_counter*tick_length-6000)/3000,true);                                  // Drive both motors forwards at full speed

      drive_motor(right_motor,255-255*(tick_counter*tick_length-6000)/3000,false);

    }

    else if (tick_counter*tick_length > 3000) {                  // Wait [3] seconds before beginning test

      drive_motor(left_motor,255-255*(tick_counter*tick_length-3000)/3000,false);                                  // Drive both motors forwards at full speed

      drive_motor(right_motor,255-255*(tick_counter*tick_length-3000)/3000,true);

    }

  }

  tick_counter ++;                                               // Increment tick counter

  delay(tick_length - (millis() % tick_length));                 // Delay the remaining milliseconds of the tick to keep tick rate constant (as long as computer fast enough)

}
