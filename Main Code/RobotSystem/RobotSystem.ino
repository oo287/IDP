// --------- Library Imports ---------                           // --------- Library Imports ---------
#include <Adafruit_MotorShield.h>                                // Import motor shield library

// --------- Variables ---------                                 // --------- Variables ---------
byte robot_test_state = 7;                                       // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details
unsigned long tick_counter = 0;                                  // Counts the number of ticks elapsed since program started running
unsigned const int tick_length = 20;                              // The length of one tick in milliseconds. Clock Frequency = 1/tick_length
unsigned long time_when_i_say_so = 0;
bool horizontal_line = false;
bool finished_dropping = false;
bool picked_up_yet = false;
byte robot_state = 0;                                            // Variable to track the stage of the problem (0=start,1=got 1 dummy,2=dropped off one dummy)

bool line_detector_1, line_detector_2, line_detector_3;          // Booleans to describe if each line sensor has a line below or not
int IR_sensor_magnitude;                                         // Output magnitude of IR sensor (for locating dummies)
int max_IR_sensor_magnitude;                                     // Maximum value of IR recorded throughout a rotation
int min_IR_sensor_magnitude;                                     // Minimum value of IR recorded throughout a rotation (used to distinguish between the dummy and background noise)
int US_magnitude;                                                // Distance recorded by the ultrasonic sensor. Return 200 if out of range (dist > 200)

int what_dummy_am_I;                                             // The dummy that is detected (0 for line, 1 for red box, 2 for blue box)

const byte left_motor_port =  3;                                 // Motor shield port that the left motor uses
const byte right_motor_port = 4;                                 // Motor shield port that the right motor uses

const byte LED1_PIN = 6;                                         // Pin used for Orange LED (LED1)
const byte LS1_PIN = A2;                                          // Pins used for Line Sensors (LS) 1, 2 and 3
const byte LS2_PIN = A1;
const byte LS3_PIN = A0;
const byte TRIG_PIN = 12;
const byte ECHO_PIN = 13;
const int IR_AMP_PIN = A3;

unsigned long sweep_start_time = 0;
bool dummy_located = false;

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

int measure_IR_amplitude(int IR_PIN, int samples) {
  // Function to record voltage amplitude of IR sensor 10% duty cycle signal
  // IR_PIN must be an analog pin for any non-binary results.
  // Wave is ~6ms long, and samples take ~ 112us, so set samples to AT LEAST >54 to ensure peak is included in measurement (preferably > 108)
  // CODE IS BLOCKING TO ALLOW FAST SAMPLING RATE - blocking time will increase linearly with samples
  
  int max_val = 0;                                                // Variable to store max. voltage measured
  int new_val;                                                // Variable to store current measurement
  for (int i=0; i<samples; i++) {
    new_val = analogRead(IR_PIN);                             // Measure current voltage from photodiode
    if (new_val > max_val) {
      max_val = new_val;                                      // Update maximum recorded value if it is new highest
    }
  }
  return max_val;
}

int take_ultrasonic_reading() {
  long duration, distance;                                       //no idea how this works but should just return distance, add some prints if i broke it
  digitalWrite(TRIG_PIN, LOW);  // Added this line 
  delayMicroseconds(2); // Added this line 
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10); // Added this line 
  digitalWrite(TRIG_PIN, LOW); 
  duration = pulseIn(ECHO_PIN, HIGH); 
  distance = (duration/2) / 29.1; 
  if (distance >= 200 || distance <= 0){
    return 200;
  } 
  else { 
    return distance;
  } 
}

void take_line_sensor_readings() {

  if (analogRead(LS1_PIN) < 500) {                               // Checks if we meet the cutoff or not (less than 500 = White)
    line_detector_1 = true;
  }
  else {
    line_detector_1 = false;
  }
  if (analogRead(LS2_PIN) < 500) {
    line_detector_2 = true;
  }
  else {
    line_detector_2 = false;
  }
  if (analogRead(LS3_PIN) < 500) {
    line_detector_3 = true;
  }
  else {
    line_detector_3 = false;
  }
}

// --------- Software Functions ---------                        // --------- Software Functions ---------
void follow_line() {                                             // Function that drives the motors and uses line sensors to move allow the line. Doesn't take inputs to stop (only call this function if the path is clear) ouput true when hits horizontal line
  take_line_sensor_readings();                                   // Update line sensor booleans
                                                                 // Default on the line, go straight ahead case
  if ((line_detector_2) and (not line_detector_1) and (not line_detector_3)){    
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 255, false);
    Serial.println("1");
  }                                                              // Central detector off line but niether side on line yet but carry on straight (this shouldn't happen normally)
  else if ((not line_detector_2) and (not line_detector_1) and (not line_detector_3)){ 
    drive_motor(left_motor, 200, false);
    drive_motor(right_motor, 200, false);
    Serial.println("2");
  }                                                              // Hit line on LHS  so steering RIGHT
  else if ((line_detector_1) and (not line_detector_3)){ 
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 200, false);
    Serial.println("3");
  }                                                              // Hit line on RHS  so steering LEFT
  else if ((not line_detector_1) and (line_detector_3)){ 
    drive_motor(left_motor, 200, false);
    drive_motor(right_motor, 255, false);
    Serial.println("4");
  }                                                              // Hit horizontal line (return TRUE)
  else if ((line_detector_2) and (line_detector_1) and (line_detector_3)){ 
    drive_motor(left_motor, 0, false);
    drive_motor(right_motor, 0, false);
    Serial.println("5");
  }
  else{                                                          // Some funky angles going on here, not an ideal case just sorta spin a'c'wise I guess
    drive_motor(right_motor, 50, false);
    drive_motor(left_motor, 50, true);
    Serial.println("6");
  }
}

bool point_towards_nearest_dummy(int sweep_time = 8000, float threshold = 0.9, int motor_sweep_speed = 30) {           // Function to: Stop, spin on the spot, turn towards first dummy seen (moving in the direction given)

  if (sweep_start_time == 0 and not dummy_located) {
    sweep_start_time = tick_counter*tick_length;
    max_IR_sensor_magnitude = 0;
    drive_motor(left_motor, 0, true);                         // Stop driving to prepare for turning around
    drive_motor(right_motor, 0, false);
  }

  if (tick_counter*tick_length < sweep_start_time + sweep_time) {
    
    drive_motor(left_motor, motor_sweep_speed, true);                        // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
    drive_motor(right_motor, motor_sweep_speed, false);
  
    IR_sensor_magnitude = measure_IR_amplitude(IR_AMP_PIN,107);                  // Take IR sensor input
  
    if (IR_sensor_magnitude > max_IR_sensor_magnitude) {           // If new max found...
      max_IR_sensor_magnitude = IR_sensor_magnitude;               // ...set max to current value
    }
  }
  else if (tick_counter*tick_length < sweep_start_time + 2*sweep_time) {

    if (not dummy_located) {

      IR_sensor_magnitude = measure_IR_amplitude(IR_AMP_PIN,107);

      if (IR_sensor_magnitude < max_IR_sensor_magnitude*threshold) {

        drive_motor(left_motor, motor_sweep_speed, false);                        // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
        drive_motor(right_motor, motor_sweep_speed, true);
        
      }
      else {
        dummy_located = true;
        sweep_start_time = 0;
        drive_motor(left_motor, 0, false);                        
        drive_motor(right_motor, 0, false);
        
      }
    }
    else if (tick_counter*tick_length > sweep_start_time + 2*sweep_time){
      drive_motor(left_motor, 0, false);                        
      drive_motor(right_motor, 0, false);
    }
  }

  
  
}

int identify_dummy(){                                           // Reads IR input signal and determines which dummy is in front of it 
  // write this when electrical sorted ir sensor
}

bool pick_up_dummy(){
  // when claw built write this and give output of when its return true when done
}

bool drop_off_dummy(){
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
  Serial.print("Motorshield successfully initialised.");

  pinMode(LED1_PIN, OUTPUT);                                     // Declare the Pin used for Orange LED (LED1) as OUTPUT
  pinMode(LS1_PIN, INPUT);                                       // Declare the line sensor pins as inputs
  pinMode(LS2_PIN, INPUT);
  pinMode(LS3_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);                                       //Declare pins for ultrasound sensor
  pinMode(ECHO_PIN, INPUT); 
  pinMode(IR_AMP_PIN,INPUT);

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
  else if (robot_test_state == 5) {
    if (tick_counter*tick_length > 3000) {
      follow_line();
    }
  }
  else if (robot_test_state == 6) {
    Serial.println(measure_IR_amplitude(IR_AMP_PIN,107));
    analogWrite(LED1_PIN,255);
  }
  else if (robot_test_state == 7) {
    if (tick_counter*tick_length > 3000) {
      if (not dummy_located) {
        point_towards_nearest_dummy(8000,0.7,150);
      }
      else {
        drive_motor(left_motor,255,false);
        drive_motor(right_motor,255,false);
      }
      
      
    }
  }
/*  else if (tick_length * tick_counter > 3000) {                    // main program
   if (robot_state == 0){                                        // starting sequence -> pick up first dummy
     if ((take_ultrasonic_reading() > 5) and (take_IR_sensor_readings() < 1024)){                        // this should drive us up over the ramp to the first dummy no idea what the ir value should be right now
       follow_line();
     }
     else {
       drive_motor(left_motor,0,true);                                    
       drive_motor(right_motor,0,true);
       if (time_when_i_say_so == 0){
         time_when_i_say_so = tick_counter*tick_length;
       }
       if (tick_length * tick_counter > time_when_i_say_so + 5000){
         which_dummy_am_I = identify_dummy();
         picked_up_yet = pick_up_dummy();
       }
       if (picked_up_yet == true){
         time_when_i_say_so = 0;
         robot_state = 1;
       }
     }
   }
 
    if (robot_state == 1){      //picked up first dummy and drop it off
      if (not follow_line()){
        horizontal_line = follow_line()
      }
      if (horizontal_line){
        finished_dropping = drop_off_dummy();
      }
      if (finished_dropping == true){
        robot_state == 2;
        finished_dropping = false
      }
    }
    if (robot_test_state ==6){      //first competetion breakaway point RESUME FROM HERE
    }
    else{
    if (robot_state == 2){     // back out of drop off
      if (time_when_i_say_so == 0){
        time_when_i_say_so = tick_counter * tick_length;
      }
      else if (tick_counter * tick_length < time_when_i_say_so + 1000){
        drive_motor(left_motor,255,true);
        drive_motor(right_motor,255,true);
      }
      else{
       robot_state = 3;
      }
    }
   
   if (robot_state == 3){                                       //find 2nd dummy
     point_towards_nearest_dummy();
     if ((take_ultrasonic_reading() > 5) and (take_IR_sensor_readings() < 1024)){                        // this should drive us to the dummy
       drive_motor(left_motor,255,false);
       drive_motor(right_motor,255,false);
       pick_up_dummy()
       if ((not line_detector_1) and (not line_detector_2) and (not line_detector_3)){
         drive_motor(left_motor,255,true);
         drive_motor(right_motor,255,true);
       }
       else{
       
       }
     }
   } 
 }
 } */

  tick_counter ++;                                               // Increment tick counter
  delay(tick_length - (millis() % tick_length));                 // Delay the remaining milliseconds of the tick to keep tick rate constant (as long as computer fast enough)
}
