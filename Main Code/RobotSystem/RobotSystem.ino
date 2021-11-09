// --------- Library Imports ---------                           // --------- Library Imports ---------


// --------- Variables ---------                                 // --------- Variables ---------
byte robot_state = 0;                                            // Variable to track the stage of the problem (0=Driving on line,1=Driving off line,2=Stopped)
byte no_dummies_rescued = 0;                                     // Number of dummies returned to their zones

byte motor1_output, motor2_output, motor3_output, motor4_output; // Voltage/Current/Whatever for controlling the speed/torque of each wheel motor. 1 = Front left, 2 = Front right, 3 = Back left, 4 = Back right

float line1_value, line2_value, line3_value, line4_value;        // Value of line sensor outputs (between 0 and 1023)
bool line1_bool, line2_bool, line3_bool, line4_bool;             // Booleans that represent if the line sensor IS over the line (calculated from lineN_value variables)

float dummy_dist = -1;                                           // Distance to dummy being tracked. -1 if carrying a dummy or not tracking a dummy

const int motor_output_high      = 100;                          // Value that represents the maximum motor output
const int motor_output_low       = 60;                           // Motor output value used for turning
const int motor_output_rev_high  = -100;                         // Motor output value used for reversing (straight)
const int motor_output_rev_low   = -60;                          // Motor output value used for reversing and turning

// --------- Functions ---------                                 // --------- Functions ---------
void follow_line() {                                             // Function that drives the motors and uses line sensors to move allow the line. Doesn't take inputs to stop (only call this function if the path is clear)
                                                                 // These if functions work out if we are on the line straight or not. The first one corresponds to perfectly straight
  if ((line2_bool and line3_bool) and not (line1_bool or line4_bool)) {
    motor1_output = motor_output_high;                           // Set motor output values for moving straight
    motor2_output = motor_output_high;
    motor3_output = motor_output_high;
    motor4_output = motor_output_high;
  }
  else if (line1_bool and not line4_bool) {                      // Line is to the left, robot too far right
    motor1_output = motor_output_low;                            // Motor outputs corresponding to turn left slightly
    motor3_output = motor_output_low;
    motor2_output = motor_output_high;
    motor4_output = motor_output_high;
  }
  else if (line4_bool and not line1_bool) {                      // Line is to the right, robot too far left
    motor1_output = motor_output_high;                           // Motor outputs corresponding to turn right slightly
    motor3_output = motor_output_high;
    motor2_output = motor_output_low;
    motor4_output = motor_output_low;
  }
  // Add more conditions here
}

// --------- Setup Function ---------                            // --------- Setup Function ---------
void setup() {                                                   // Function that runs on power-up/RESET
  robot_state = 0;                                               // Reset robot state to default
}


// --------- Main Loop ---------                                 // --------- Main Loop ---------
void loop() {                                                    // Function that runs repeatedly whilst the robot is on
  follow_line();
}
