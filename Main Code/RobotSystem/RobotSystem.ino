// --------- Library Imports ---------                           // --------- Library Imports ---------
#include <Adafruit_MotorShield.h>                                // Import motor shield library
#include <Servo.h>                                               // Import servo library


// --------- Important Variables ---------                       // --------- Important Variables ---------
int robot_test_state = 12;                                       // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details
int robot_state = 0;                                             // Variable to track the stage of the problem (0=start,1=got 1 dummy,2=dropped off one dummy)
unsigned long tick_counter = 0;                                  // Counts the number of ticks elapsed since program started running
unsigned const int tick_length = 20;                             // The length of one tick in milliseconds. Clock Frequency = 1/tick_length. Since IR sensor takes ~12ms, 20ms currently used


// --------- Hardware Constants ---------                        // --------- Hardware Constants ---------
const int LM_port =  3;                                          // Motor shield port that the left motor uses
const int RM_port = 4;                                           // Motor shield port that the right motor uses

const int LED1_PIN = 6;                                          // Pin used for Orange LED (LED1)
const int LED2_PIN = 3;                                          // Pin used for Red LED (LED2)
const int LED3_PIN = 5;                                          // Pin used for Green LED (LED2)
const int LS1_PIN = A0;                                          // Pins used for Line Sensors (LS) 1, 2 and 3
const int LS2_PIN = A2;
const int LS3_PIN = A1;
const int TRIG_PIN = 12;                                         // TRIG and ECHO pins are used for the ultrasonic sensor
const int ECHO_PIN = 13;
const int IR_AMP_PIN = A3;                                       // Pin used to read the amplitude of the IR signal
const int MOD_PIN = 7;                                           // IR to detect modulation and hence identify dummeis
const int SERVO_1_PIN = 9;                                       // Claw grab servo
const int SERVO_2_PIN = 10;                                      // Claw lift servo


// --------- Sensor Variables ---------                          // --------- Sensor Variables ---------
bool line_1, line_2, line_3;                                     // Booleans to describe if each line sensor has a line below or not
int IR_amplitude;                                                // Output magnitude of IR sensor (for locating dummies)
int max_IR_amplitude;                                            // Maximum value of IR recorded throughout a rotation
int US_amplitude;                                                // Distance recorded by the ultrasonic sensor in cm. Return 200 if out of range (dist > 200cm)


// --------- Logic Variables ---------                           // --------- Logic Variables ---------
bool horizontal_line = false;
bool finished_dropping = false;
bool picked_up_yet = false;

int what_dummy_am_I;                                             // The dummy that is detected (0 for line, 1 for red box, 2 for blue box)
bool dummy_located = false;                                      // If a dummy has been spotted using IR amplitude or not

int scared = 2;

int claw_servo_angle = 20;                                       // Angle for claw servo
int lift_servo_angle = 20;                                       // Angle for lift servo


// --------- Timer/Timing Variables ---------                    // --------- Timer/Timing Variables ---------
unsigned long delay_5s_start_time = 0;
unsigned long sweep_start_time = 0;                              // Timer used to time how long to sweep when searching for a dummy with IR amplitude sensor
unsigned long adjust_start_time = 0;                             // Timer used for the timing of the sweep in the adjust action 
int home_adjust_distance = 20;
int home_final_distance = 4;    
unsigned long turn_timer = 0;                                    // Timer used for turning 360, 180 or 90 degrees

const int full_360_time = 6000;                                  // Time taken for full 360 at set motor speed
const int motor_360_speed = 100;                                 // Motor speed used for turning a full 360


// --------- Motor Initialisation ---------                      // --------- Motor Initialisation ---------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();              // Create motorshield object called 'AFMS' with default I2C address. Put I2C address in brackets if different address needed
Adafruit_DCMotor *left_motor = AFMS.getMotor(LM_port);           // Initialise left  motor using assigned motor shield port
Adafruit_DCMotor *right_motor = AFMS.getMotor(RM_port);          // Initialise right motor using assigned motor shield port

Servo claw_servo;                                                // Servo for grabbing dummy
Servo lift_servo;                                                // Servo for lifting claw

claw_servo.attach(SERVO_1_PIN);
lift_servo.attach(SERVO_2_PIN);


// --------- Electrical Functions ---------                      // --------- Electrical Functions ---------
void drive_motor(Adafruit_DCMotor* motor, int spd, bool rev) {   // Electrical function that drives a single motor. motor: 0=Left,1=Right; spd: 0-255; rev: bool, true = Reverse
  
  if (rev) {                                                     // If motor needs to run in reverse
    motor->run(BACKWARD);                                        // Run in reverse
    }
  else {
    motor->run(FORWARD);                                         // Run forwards
  }
  motor->setSpeed(spd);                                          // Set motor speed to spd inputted

  if (spd != 0) {                                                // If motor running at non-zero speed
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

                                                                 // Function to record voltage amplitude of IR sensor 10% duty cycle signal
                                                                 // IR_PIN must be an analog pin for any non-binary results.
                                                                 // Wave is ~6ms long, and samples take ~ 112us, so set samples to AT LEAST >54 to ensure peak is included in measurement (preferably > 108)
int measure_IR_amplitude(int IR_PIN, int samples) {              // CODE IS BLOCKING TO ALLOW FAST SAMPLING RATE - blocking time will increase linearly with samples
  
  int max_val = 0;                                               // Variable to store max. voltage measured
  int new_val;                                                   // Variable to store current measurement
  for (int i=0; i<samples; i++) {
    new_val = analogRead(IR_PIN);                                // Measure current voltage from photodiode
    if (new_val > max_val) {
      max_val = new_val;                                         // Update maximum recorded value if it is new highest
    }
  }
  return max_val;
}

int take_ultrasonic_reading() {                                  // Function to return the distance (in cm) measured by the ultrasonic sensor
  
  long duration, distance;                                       
  digitalWrite(TRIG_PIN, LOW);                                   // TRIG pin sends a pulse out...
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW); 
  duration = pulseIn(ECHO_PIN, HIGH);                            // ...and the ECHO pin records the reflected pulse and measures the time to find the distance
  
  distance = (duration/2) / 29.1; 
  
  if (distance >= 200 || distance <= 0){                         // If distance more than 200, just return 200 (max output)
    return 200;
  } 
  else { 
    return distance;                                             // If between 0 and 200, return the actual value
  } 
}

void read_line_sensors() {                                       // Update the booleans line_N with whether they are on the line or not (cutoff = 500)

  if (analogRead(LS1_PIN) < 500) {                               // Checks if we meet the cutoff or not (less than 500 = White)
    line_1 = true;
  }
  else {
    line_1 = false;
  }
  if (analogRead(LS2_PIN) < 500) {
    line_2 = true;
  }
  else {
    line_2 = false;
  }
  if (analogRead(LS3_PIN) < 500) {
    line_3 = true;
  }
  else {
    line_3 = false;
  }
}


// --------- Software Functions ---------                        // --------- Software Functions ---------
bool follow_line() {                                             // Function that drives the motors and uses line sensors to move allow the line. Doesn't take inputs to stop (only call this function if the path is clear) ouput true when hits horizontal line
  
  read_line_sensors();                                           // Update line sensor booleans
  
  if ((line_2) and (not line_1) and (not line_3)){               // Default on the line, go straight ahead case
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 255, false);
  }
  else if ((not line_2) and (not line_1) and (not line_3)){      // Central detector off line but niether side on line yet but carry on straight (this shouldn't happen normally)
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 255, false);
  }                                                              
  else if ((line_1) and (not line_3)){                           // Hit line on LHS  so steering RIGHT
    drive_motor(left_motor, 150, false);
    drive_motor(right_motor, 255, false);
  }                                                              
  else if ((not line_1) and (line_3)){                           // Hit line on RHS  so steering LEFT
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 150, false);
  }                                                              
  else if ((line_2) and (line_1) and (line_3)){                  // Hit horizontal line (return TRUE)
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 255, false);
    return true;
  }
  else{                                                          // Some funky angles going on here, not an ideal case just sorta spin a'c'wise I guess
    drive_motor(right_motor, 255, false);
    drive_motor(left_motor, 255, true);
  }
  return false;
}

                                                                 // Function to: Stop, spin on the spot, turn towards brighest dummy seen, stop again and set dummy_located = true
void point_towards_nearest_dummy(int sweep_time = 6000, float threshold = 0.7, int motor_sweep_speed = 150) {

  if (sweep_start_time == 0 and not dummy_located) {             // If haven't started sweeping yet (first time activating function) and haven't already found dummy
    sweep_start_time = tick_counter*tick_length;                 // Set start time to current time
    max_IR_amplitude = 0;                                        // Initialise IR amplitude at 0
    drive_motor(left_motor, 0, false);                           // Stop driving to prepare for turning around
    drive_motor(right_motor, 0, false);
  }

  if (tick_counter*tick_length < sweep_start_time+sweep_time) {  // If still within first sweep (sweep_start_time -> sweep_start_time + sweep_time)
    
    drive_motor(left_motor, motor_sweep_speed, true);            // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
    drive_motor(right_motor, motor_sweep_speed, false);
  
    IR_amplitude = measure_IR_amplitude(IR_AMP_PIN,107);         // Take IR sensor input
  
    if (IR_amplitude > max_IR_amplitude) {                       // If new max found...
      max_IR_amplitude = IR_amplitude;                           // ...set max to current value
    }
  }                                                              // If outside of first sweep and within second. The second sweep lasts no more than 3x as long as the first sweep
  else if (tick_counter*tick_length < sweep_start_time + 4*sweep_time) {

    if (not dummy_located) {                                     // If haven't found dummy yet

      IR_amplitude = measure_IR_amplitude(IR_AMP_PIN,107);       // Read IR amplitude

      if (IR_amplitude < max_IR_amplitude*threshold) {           // If IR value hasn't reached cutoff (threshold*max_IR_amplitude), then continue turning, searching

        drive_motor(left_motor, motor_sweep_speed, false);       // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
        drive_motor(right_motor, motor_sweep_speed, true);
        
      }
      else {                                                     // If reached cutoff (threshold*max_IR_amplitude), then declare dummy found and stop turning, reset sweep_start_time
        dummy_located = true;                                    // Dummy found
        sweep_start_time = 0;                                    // Reset sweep start time
        drive_motor(left_motor, 0, false);                       // Stop driving
        drive_motor(right_motor, 0, false);                      // Stop driving
        
      }
    }                                                            // If outside 2nd sweep and no dummy found, just stop (and have a little cry)
    else if (tick_counter*tick_length > sweep_start_time + 4*sweep_time){
      drive_motor(left_motor, 0, false);                         // Stop driving
      drive_motor(right_motor, 0, false);                        // Stop driving
    }
  }
}

void home_dummy() {                                              // Algorithm to drive forwards and adjust so perfectly* in front of dummy

  US_amplitude = take_ultrasonic_reading();                      // Update ultrasonic distance reading

  if (US_amplitude > home_adjust_distance) {                     // If too far away to bother adjusting...
    drive_motor(left_motor, 200, false);                         // ...just drive
    drive_motor(right_motor, 200, false);
  }
  else {
    if (adjust_start_time == 0) {                                // If within adjust distance, activate this once:
      adjust_start_time = tick_counter*tick_length;              // Set start time to current time
      dummy_located = false;                                     // Declare that dummy not found
    }
    else if (tick_counter*tick_length<adjust_start_time+1000) {  // For [1] second(s) after starting:
      drive_motor(left_motor, 100, false);                       // Turn clockwise slowly
      drive_motor(right_motor, 100, true);
    }
    else if (not dummy_located) {                                // After the first sweep, start sweeping for dummy slowly and only in the small angle range
      point_towards_nearest_dummy(2000, 0.95, 100);              // [2] second(s), High threshold, slowly
    }
    else if (dummy_located) {                                    // After finding dummy again...
      if (US_amplitude > home_final_distance) {                  // ...drive until at final distance and stop
        drive_motor(left_motor, 150, false);
        drive_motor(right_motor, 150, false);
      }
      else {
        drive_motor(left_motor, 0, false);
        drive_motor(right_motor, 0 , false);
      }
    }
  } 
}

bool turn(int angle, bool clockwise) {                           // Turns a set angle. Should be accurate since uses carefully calibrated motor speed + turn time. Returns true when done

  if (turn_timer == 0) {                                         // If haven't started turning yet

    turn_timer = tick_counter*tick_length;                       // Set current time to turn start time
    
  }
  else if (tick_counter*tick_length < turn_timer + (angle/full_360_time)) {
    drive_motor(left_motor, motor_360_speed, not clockwise);     // Drive for the correct proportion of a 360 turn at the calibrated speed
    drive_motor(right_motor, motor_360_speed, clockwise);
    return false;                                                // Not done yet so return false
  }
  else {
    drive_motor(left_motor, 0, false);                           // Stop motors
    drive_motor(right_motor, 0, false);
    return true;                                                 // Done so return true
  }
}

int identify_dummy(){                                            // Reads IR input signal and determines which dummy is in front of it 
  // write this when electrical sorted ir sensor
}

bool pick_up_dummy(){                                            // Drive servos to pick up dummy. Return true when finished
  // when claw built write this and give output of when its return true when done
}

bool drop_off_dummy(){                                           // Drive servos to released dummy. Return true when finished
  // write this when claw built return true when done
}
void approach_identify_grab(){
  //idk if we need this but its here if we do
}


// --------- Setup Function ---------                            // --------- Setup Function ---------
void setup() {                                                   // Function that runs on power-up/RESET
  Serial.begin(9600);                                            // Start Serial to print debug info

  while (!AFMS.begin()) {                                        // Wait until motorshield successfully initialised
    Serial.println("Failed to find motorshield.");
    delay(1000);                 
  }
  Serial.print("Motorshield successfully initialised.");         // Infamous message that will haunt your nightmares (displays repeatedly if Arduino crashing (check drive_motor() uses!))

  pinMode(LED1_PIN, OUTPUT);                                     // Declare the Pin used for Orange LED (LED1) as OUTPUT
  pinMode(LS1_PIN, INPUT);                                       // Declare the line sensor pins as inputs
  pinMode(LS2_PIN, INPUT);
  pinMode(LS3_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);                                     // Use TRIG pin as output for ultrasonic sensor (this pin gives an ultrasonic pulse)
  pinMode(ECHO_PIN, INPUT);                                      // Use ECHO pin as input for ultrasonic sensor (this pin reads the echoed ultrasonic pulse from the TRIG pin)
  pinMode(IR_AMP_PIN,INPUT);                                     // Pin for measuring the amplitude of the IR sensor for use in locating dummies

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
  if (robot_test_state == -1) {                                  // Test -1: Display all sensor/input values
    Serial.print("IR Amp: ");
    Serial.print(measure_IR_amplitude(IR_AMP_PIN,107));          // IR Amplitude
    Serial.print(" / 1023, US Dist: ");
    Serial.print(take_ultrasonic_reading());                     // Ultrasonic distance
    read_line_sensors();                                         // Update line sensors
    Serial.print(" cm, LS1: ");
    Serial.print(line_1);                                        // Print each line sensor boolean (true = on line, false = off line)
    Serial.print(",");
    Serial.print(analogRead(LS1_PIN));
    Serial.print(", LS2: ");
    Serial.print(line_2);
    Serial.print(",");
    Serial.print(analogRead(LS2_PIN));
    Serial.print(", LS3: ");
    Serial.print(line_3);
    Serial.print(",");
    Serial.println(analogRead(LS3_PIN));
  }
  else if (robot_test_state == 1) {                              // Test 1: Spin the wheels without control of any kind
    if (tick_counter*tick_length > 3000) {                       // Wait [3] seconds before beginning test
      drive_motor(left_motor,255,false);                         // Use the electrical function to spin each motor at max speed
      drive_motor(right_motor,0,false);
    }
  }
  else if (robot_test_state == 2 or robot_test_state == 3) {     // Test 2/3: Drive forwards for a fixed time (to give ~1m forwards)
    if (tick_counter*tick_length > 7000) {                       // Drive for [4] seconds
      drive_motor(left_motor,0,false);                           // Stop motors after test complete
      drive_motor(right_motor,0,false);
    }
    else if (tick_counter*tick_length > 3000) {                  // Wait [3] seconds before beginning test
      drive_motor(left_motor,255,false);                         // Drive both motors forwards at full speed
      drive_motor(right_motor,255,false);
    }
  }
  else if (robot_test_state == 4) {                              // Test 4: 360 turn in each direction, then reverse for ~1m
    if (tick_counter*tick_length > 13000) {                      // Stop motors after test complete
      drive_motor(left_motor,0,true);                                    
      drive_motor(right_motor,0,true);
    }
    else if (tick_counter*tick_length > 9000) {                  // Reverse for [4] seconds = ~1m
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
  else if (robot_test_state == 5) {                              // Test 5: Crash into dummy. Point at nearest dummy using IR and drive forwards. don't. stop.
    if (tick_counter*tick_length > 3000) {
      if (not dummy_located) {
        point_towards_nearest_dummy(6000,0.7,150);
      }
      else {
        drive_motor(left_motor,255,false);
        drive_motor(right_motor,255,false);
      }
    }
  }
  else if (robot_test_state == 9) {
    if (tick_counter*tick_length > 3000) {
      if (tick_counter*tick_length % 6000 > 3000) { 
        
      }
      else if (tick_counter*tick_length % 6000 <= 3000) {
        claw_servo.write(tick_counter*tick_length%6000); // Unfinished
      }
    }
  }
  else if (robot_test_state == 12) {                             // Test 12: Follow line
    if (tick_counter*tick_length > 3000) {                       // Wait [3] seconds before beginning test
      follow_line(true);                                         // Just run the follow_line() function (forwards)
    }
  }
  else if (robot_test_state == 20) {
    if (scared == 2) {
      scared = 0;
    }
    if (tick_counter*tick_length > 3000) {
      if (measure_IR_amplitude < 200 and take_ultrasonic_reading > 20 and scared == 0) {
        follow_line();
      }
      else {
        if (scared == 0) {
          scared = 1;
        }
        follow_line(true);
      }
    }
  }
      
      

//  else if (tick_length * tick_counter > 3000) {                    // main program
//   if (robot_state == 0){                                        // starting sequence -> pick up first dummy
//     if ((take_ultrasonic_reading() > 5) and (take_IR_sensor_readings() < 1024)){                        // this should drive us up over the ramp to the first dummy no idea what the ir value should be right now
//       follow_line();
//     }
//     else {
//       drive_motor(left_motor,0,true);                                    
//       drive_motor(right_motor,0,true);
//       if (time_when_i_say_so == 0){
//         time_when_i_say_so = tick_counter*tick_length;
//       }
//       if (tick_length * tick_counter > time_when_i_say_so + 5000){
//         which_dummy_am_I = identify_dummy();
//         picked_up_yet = pick_up_dummy();
//       }
//       if (picked_up_yet){
//         time_when_i_say_so = 0;
//         robot_state = 1;
//         picked_up_yet = false;
//       }
//     }
//   }
// 
//    if (robot_state == 1){      //picked up first dummy and drop it off
//      if (not follow_line()){
//        horizontal_line = follow_line()
//      }
//      if (horizontal_line){
//        finished_dropping = drop_off_dummy();
//      }
//      if (finished_dropping){
//        robot_state == 2;
//        finished_dropping = false;
//      }
//    }
//    if (robot_test_state == 6){      //first competetion breakaway point RESUME FROM HERE
//      // turn 180 and drive back to start
//    }
//    else{
//      if (robot_state == 2){     // back out of drop off
//        if (delay_5s_start_time == 0){
//          delay_5s_start_time = tick_counter * tick_length;
//        }
//        else if (tick_counter * tick_length < delay_5s_start_time + 3000){
//          drive_motor(left_motor,255,true);
//          drive_motor(right_motor,255,true);
//        }
//        else{
//          robot_state = 3;
//          delay_5s_start_time =0;
//        }
//      }
//    
//    if (robot_state == 3){                                       //find 2nd dummy
//      point_towards_nearest_dummy();
//      if ((take_ultrasonic_reading() > 5) and (take_IR_sensor_readings() < 1024)){                        // this should drive us to the dummy
//        drive_motor(left_motor,255,false);
//        drive_motor(right_motor,255,false);
//        pick_up_dummy()
//        if ((not line_1) and (not line_2) and (not line_3)){      // back up untill we hit line
//          drive_motor(left_motor,255,true);
//          drive_motor(right_motor,255,true);
//        }
//        else{
//         // spin untill on line facing either way, if ultrasound < 1m do a U turn, if >1m go straight on, need to do spin 180 code and from the reverse position spin and go forward. also follow line reverse
//        }
//      }
//    } 
//  }
// } 

  tick_counter ++;                                               // Increment tick counter
  delay(tick_length - (millis() % tick_length));                 // Delay the remaining milliseconds of the tick to keep tick rate constant (as long as computer fast enough)
}
