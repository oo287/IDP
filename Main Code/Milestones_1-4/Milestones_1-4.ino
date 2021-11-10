// --------- Library Imports ---------                           // --------- Library Imports ---------
#include <Adafruit_MotorShield.h>                                // Import motor shield library

// --------- Variables ---------                                 // --------- Variables ---------
byte robot_test_state = 0;                                       // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details

byte robot_state = 0;                                            // Variable to track the stage of the problem (0=Driving on line,1=Driving off line,2=Stopped)
byte no_dummies_rescued = 0;                                     // Number of dummies returned to their zones

byte left_motor_output, right_motor_output                       // Voltage/Current/Whatever for controlling the speed/torque of each wheel motor. 1 = Front left, 2 = Front right, 3 = Back left, 4 = Back right

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
  left_motor  -> setSpeed(0);
  right_motor -> setSpeed(0);
  
}


// --------- Main Loop ---------                                 // --------- Main Loop ---------
void loop() {                                                    // Function that runs repeatedly whilst the robot is on
  
}
