// --------- Library Imports ---------                           // --------- Library Imports ---------
#include <Adafruit_MotorShield.h>                                // Import motor shield library

// --------- Variables ---------                                 // --------- Variables ---------
unsigned byte robot_test_state = 4;                              // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details
unsigned long tick_counter = 0;                                  // Counts the number of ticks elapsed since program started running
unsigned const int tick_length = 5;                              // The length of one tick in milliseconds. Clock Frequency = 1/tick_length

byte robot_state = 0;                                            // Variable to track the stage of the problem (0=Driving on line,1=Driving off line,2=Stopped)
byte no_dummies_rescued = 0;                                     // Number of dummies returned to their zones

int line_detector_1, line_detector_2, line_detector_3;           // Output of line detector (0-1023) 1 LHS 2 middle 3 RHS
int IR_sensor_magnitude;                                         // Output magnitude of IR sensor (for locating dummies)
int max_IR_sensor_magnitude;                                     // Maximum value of IR recorded throughout a rotation
int min_IR_sensor_magnitude;                                     // Minimum value of IR recorded throughout a rotation (used to distinguish between the dummy and background noise)

const byte left_motor_port =  3;                                 // Motor shield port that the left motor uses
const byte right_motor_port = 4;                                 // Motor shield port that the right motor uses

const byte LED1_PIN = 6;                                         // Pin used for Orange LED (LED1)

// --------- Motor Initialisation ---------                      // --------- Motor Initialisation ---------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();              // Create motorshield object called 'AFMS' with default I2C address. Put I2C address in brackets if different address needed
Adafruit_DCMotor *left_motor = AFMS.getMotor(left_motor_port);   // Initialise left  motor using assigned motor shield port
Adafruit_DCMotor *right_motor = AFMS.getMotor(right_motor_port); // Initialise right motor using assigned motor shield port

// --------- Electrical Functions ---------                      // --------- Electrical Functions ---------
void drive_motor(Adafruit_DCMotor* motor, int spd, bool rev) {   // Electrical function that drives a single motor. motor: 0=Left,1=Right; spd: 0-255; rev: bool, true = Reverse
  
  if (rev) {                                                     // If motor needs to run in reverse
    motor->run(BACKWARD);                                        // Run in reverse
    }
  else {
    motor->run(FORWARD);                                         // Run forwards
  }
  motor->setSpeed(spd);                                          // Set motor speed to spd inputted

  if (spd != 0) {                                                // If motor running
    if (tick_counter*tick_length % 500 < 250) {                  // Use tick_counter*tick_length to set 500ms frequency for LED flashing
      analogWrite(LED1_PIN,255);                                 // Write HIGH output to LED Pin 1 (orange LED)
    }
    else if (tick_counter*tick_length % 500 > 249) {             // If in second half of 500ms time period (since 50% duty cycle)
      analogWrite(LED1_PIN,0);                                   // Write LOW output to LED Pin 1 (orange LED)
    }
  }
  else {
    analogWrite(LED1_PIN,0);                                     // Turn orange LED off if not moving (writing spd = 0)
  } 
}

int take_IR_sensor_reading() {                                   // Electrical function that returns an integer corresponding to the magnitude of the peaks of the IR input from the sensor

  // ELECTRICAL WRITE THIS FUNCTION (PLEASE)
 
}

bool take_line_sensor_reading(byte line_sensor_number = 0) {     // Electrical function that returns an integer (byte) for what colour the line below is (true=white,false=black) (0=Left,1=Center,2=Right sensor)

  // ELECTRICAL WRITE THIS FUNCTION (PLEASE) 
  
}

int take_ultrasonic_reading() {

  // ELECTRICAL WRITE THIS FUNCTION (PLEASE)
  
}

// --------- Software Functions ---------                        // --------- Software Functions ---------
void follow_line() {                                             // Function that drives the motors and uses line sensors to move allow the line. Doesn't take inputs to stop (only call this function if the path is clear)
line_detector_1 = take_line_sensor_reading(0)                    // initiate line sensor variable (1=L 2=m 3=R)
line_detector_2 = take_line_sensor_reading(1)
line_detector_3 = take_line_sensor_reading(2)

                                                                 // Default on the line, go straight ahead case
  if ((line_detector_2 == true) and (line_detector_1 == false and (line_detector_3 == false)){    
    drive_motor(left_motor_port, 255, false);
    drive_motor(right_motor_port, 255, false);
  }                                                              // Central detector off line but niether side on line yet but carry on straight (this shouldn't happen normally)
  else if ((line_detector_2 == false) and (line_detector_1 == false) and (line_detector_3 == false)){ 
    drive_motor(left_motor_port, 200, false);
    drive_motor(right_motor_port, 200, false);
  }                                                              // Hit line on LHS  so steering RIGHT
  else if ((line_detector_1 == true) and (line_detector_3 == false){ 
    drive_motor(left_motor_port, 255, false);
    drive_motor(right_motor_port, 200, false);
  }                                                              // Hit line on RHS  so steering LEFT
  else if ((line_detector_1 == false) and (line_detector_3 == true)){ 
    drive_motor(left_motor_port, 200, false);
    drive_motor(right_motor_port, 255, false);
  }                                                              // Hit horizontal line (STOP)
  else if ((line_detector_2 == true) and (line_detector_1 == true) and (line_detector_3 == true)){ 
    drive_motor(left_motor_port, 0, false);
    drive_motor(right_motor_port, 0, false);
  }
  else{                                                          // Some funky angles going on here, not an ideal case just sorta spin a'c'wise I guess
    drive_motor(right_motor_port, 100, false);
    drive_motor(left_motor_port, 100, true);
  }
}

bool point_towards_nearest_dummy(bool clockwise=true){           // Function to: Stop, spin on the spot, turn towards first dummy seen (moving in the direction given)

  drive_motor(left_motor_port, 0, true);                         // Stop driving to prepare for turning around
  drive_motor(right_motor_port, 0, false);
  
  drive_motor(left_motor_port, 50, true);                        // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
  drive_motor(right_motor_port, 50 , false);

  IR_sensor_magnitude = take_IR_sensor_reading();                // Take IR sensor input

  if (IR_sensor_magnitude > max_IR_sensor_magnitude) {           // If new max found...
    max_IR_sensor_magnitude = IR_sensor_magnitude;               // ...set max to current value
  }
  if (IR_sensor_magnitude < min_IR_sensor_magnitude) {           // If new min found...
    min_IR_sensor_magnitude = IR_sensor_magnitude;               // ...set min to current value
  }
  // UNFINISHED (OLLIE)
}

// --------- Setup Function ---------                            // --------- Setup Function ---------
void setup() {                                                   // Function that runs on power-up/RESET
  Serial.begin(9600);                                            // Start Serial to print debug info

  while (!AFMS.begin()) {                                        // Wait until motorshield successfully initialised
    Serial.println("Failed to find motorshield.");
    delay(1000);                 
  }
  Serial.print("Motorshield successfully initialised.");

  pinMode(LED1_PIN, OUTPUT);                                     // Declare the Pin used for Orange LED (LED1) as OUTPUT

  robot_state = 0;                                               // Reset robot state to default

  left_motor->setSpeed(0);                                       // Initialise left motor
  left_motor->run(FORWARD);
  left_motor->run(RELEASE); 

  right_motor->setSpeed(0);                                      // Initialise right motor
  right_motor->run(FORWARD);
  right_motor->run(RELEASE); 
}

// --------- Main Loop ---------                                 // --------- Main Loop ---------
void loop() {                                                    // Function that runs repeatedly whilst the robot is on
  if (robot_test_state == 1) {                                   // Test 1: Spin the wheels without control of any kind
    if (tick_counter*tick_length > 3000) {                       // Wait [3] seconds before beginning test
      drive_motor(left_motor,255,false);                         // Use the electrical function to spin each motor at max speed
      drive_motor(right_motor,255,false);
    }
  }
  else if (robot_test_state == 2 or robot_test_state == 3) {     // Test 2/3: Drive forwards for a fixed time (to give ~1m forwards)
    if (tick_counter*tick_length > 7000) {                       // Drive for [2] seconds
      drive_motor(left_motor,0,false);                           // Stop motors after test complete
      drive_motor(right_motor,0,false);
    }
    else if (tick_counter*tick_length > 3000) {                  // Wait [3] seconds before beginning test
      drive_motor(left_motor,255,false);                         // Drive both motors forwards at full speed
      drive_motor(right_motor,255,false);
    }
  }
  else if (robot_test_state == 4) {                              // Test 4: 360
    if (tick_counter*tick_length > 11000) {                      // Stop motors after test complete
      drive_motor(left_motor,0,true);                                    
      drive_motor(right_motor,0,true);
    }
    else if (tick_counter*tick_length > 9000) {                  // Reverse for [2] seconds
      drive_motor(left_motor,255,true);                                    
      drive_motor(right_motor,255,true);
    }
    else if (tick_counter*tick_length > 6000) {                  // Spin Counter-clockwise, slowing down to 0 at the end of the 3-second turn
      drive_motor(left_motor,255-255*(tick_counter*tick_length-6000)/3000,true);                                  
      drive_motor(right_motor,255-255*(tick_counter*tick_length-6000)/3000,false);
    }
    else if (tick_counter*tick_length > 3000) {                  // Wait [3] seconds before beginning test
                                                                 // Spin Clockwise, slowing down to 0 at the end of the 3-second turn
      drive_motor(left_motor,255-255*(tick_counter*tick_length-3000)/3000,false);                                  
      drive_motor(right_motor,255-255*(tick_counter*tick_length-3000)/3000,true);
    }
  }
  
  tick_counter ++;                                               // Increment tick counter
  delay(tick_length - (millis() % tick_length));                 // Delay the remaining milliseconds of the tick to keep tick rate constant (as long as computer fast enough)
}
