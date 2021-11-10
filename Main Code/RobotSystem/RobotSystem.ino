// --------- Library Imports ---------                           // --------- Library Imports ---------
#include <Adafruit_MotorShield.h>                                // Import motor shield library

// --------- Variables ---------                                 // --------- Variables ---------
byte robot_test_state = 0;                                       // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details

long tick_counter = 0;                                           // Counts the number of ticks elapsed since program started running
const int tick_length = 5;                                       // The length of one tick in milliseconds. Clock Frequency = 1/tick_length

byte robot_state = 0;                                            // Variable to track the stage of the problem (0=Driving on line,1=Driving off line,2=Stopped)
byte no_dummies_rescued = 0;                                     // Number of dummies returned to their zones

byte left_motor_output, right_motor_output;                      // Voltage/Current/Whatever for controlling the speed/torque of each wheel motor. 1 = Front left, 2 = Front right, 3 = Back left, 4 = Back right

float line1_value, line2_value, line3_value, line4_value;        // Value of line sensor outputs (between 0 and 1023)
bool line1_bool, line2_bool, line3_bool, line4_bool;             // Booleans that represent if the line sensor IS over the line (calculated from lineN_value variables)

float dummy_dist = -1;                                           // Distance to dummy being tracked. -1 if carrying a dummy or not tracking a dummy

const int motor_output_high      = 100;                          // Value that represents the maximum motor output
const int motor_output_low       = 60;                           // Motor output value used for turning
const int motor_output_rev_high  = -100;                         // Motor output value used for reversing (straight)
const int motor_output_rev_low   = -60;                          // Motor output value used for reversing and turning

const byte left_motor_port =  1;                                 // Port of the motor shield that the left motor uses
const byte right_motor_port = 2;                                 // Port of the motor shield that the right motor uses

// --------- Motor Initialisation ---------                      // --------- Motor Initialisation ---------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();              // Create motorshield object called 'AFMS' with default I2C address. Put I2C address in brackets if different address needed
Adafruit_DCMotor *left_motor = AFMS.getMotor(left_motor_port);   // Initialise left  motor using assigned motor shield port
Adafruit_DCMotor *right_motor = AFMS.getMotor(right_motor_port); // Initialise right motor using assigned motor shield port

// --------- Functions ---------                                 // --------- Functions ---------
void drive_motor(int motor, int spd, bool rev) {                 // Electrical function that drives a single motor. motor: 0=Left,1=Right; spd: 0-255; rev: bool, true = Reverse
  // ELECTRICAL FUNCTION HERE INCL. LED FLASH (USE GLOBAL 'tick_counter')
}

void follow_line() {                                             // Function that drives the motors and uses line sensors to move allow the line. Doesn't take inputs to stop (only call this function if the path is clear)
  
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
  left_motor  -> setSpeed(0);                                    // Set each motor to be at speed 0 to start with
  right_motor -> setSpeed(0);
  
}


// --------- Main Loop ---------                                 // --------- Main Loop ---------
void loop() {                                                    // Function that runs repeatedly whilst the robot is on
  if (robot_test_state == 1) {                                   // Test 1: Spin the wheels without control of any kind
    if (tick_counter*tick_length > 3000) {                       // Wait 3 seconds before beginning test 
      drive_motor(0,255,false);                                    // Use the electrical function to spin each motor at max speed
      drive_motor(1,255,false); 
    }
  }
  else if (robot_test_state == 2 or robot_test_state == 3) {        // Test 2/3: Drive forwards for a fixed time (to give ~1m forwards)
    if (tick_counter*tick_length > 5000) {                       // Drive for [2] seconds
      drive_motor(0,0,false);                                    // Stop motors after test complete
      drive_motor(1,0,false);
    }
    else if (tick_counter*tick_length > 3000) {                  // Wait [3] seconds before beginning test
      drive_motor(0,255,false);                                  // Drive both motors forwards at full speed
      drive_motor(1,255,false);
    }
    
  }
  tick_counter ++;                                               // Increment tick counter
  delay(tick_length - (millis() % tick_length));                 // Delay the remaining milliseconds of the tick to keep tick rate constant (as long as computer fast enough)
}
