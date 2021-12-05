// --------- Library Imports ---------                           // --------- Library Imports ---------
#include <Adafruit_MotorShield.h>                                // Import motor shield library
#include <Servo.h>                                               // Import servo library


// --------- Important Variables ---------                       // --------- Important Variables ---------
int robot_test_state = 0;                                        // Variable to control if the robot runs a test or not. 0 = Normal, 1 = Test 1, 2 = Test 2 etc. See tests.txt for details
int robot_state = 0;                                             // Variable to track the stage of the problem (0=start,1=got 1 dummy,2=dropped off one dummy)
int robot_sub_state = 0;
unsigned long tick_counter = 0;                                  // Counts the number of ticks elapsed since program started running
unsigned const int tick_length = 20;                             // The length of one tick in milliseconds. Clock Frequency = 1/tick_length. Since IR sensor takes ~12ms, 20ms currently used
bool program_started = false;                                    // Bool to store if the program start button has been pressed yet or not
bool using_servos = false;                                       // Bool to track if we are using the servos
bool main_test_started = false;                                  // Bool to track tests number 100+
int stopping_distance = 6;                                       // Distance to stop away from dummies before grabbing (in cm)

int readings_mod = 0;                                            // 3 Counters for taking a mode of the IR modulation LED input
int readings_mix = 0;
int readings_unm = 0;

bool dummy_identity_decided = false;                             // Bool to track if the mode of IR modulation had finished and a dummy number decided

// --------- Hardware Constants ---------                        // --------- Hardware Constants ---------
const int LM_port =  3;                                          // Motor shield port that the left motor uses
const int RM_port = 4;                                           // Motor shield port that the right motor uses
const int LED1_PIN = 0;                                          // Pin used for Orange LED (LED1)
const int LED2_PIN = 1;                                          // Pin used for Red LED (LED2)
const int LED3_PIN = 2;                                          // Pin used for Green LED (LED2)
const int LS1_PIN = A0;                                          // Pins used for Line Sensors (LS) 1, 2 and 3
const int LS2_PIN = A2;
const int LS3_PIN = A1;
const int TRIG_PIN = 13;                                         // TRIG and ECHO pins are used for the ultrasonic sensor
const int ECHO_PIN = 12;
const int IR_AMP_PIN = A3;                                       // Pin used to read the amplitude of the IR signal
const int IR_MOD_PIN = 5;                                        // IR to detect modulation and hence identify dummeis
const int SERVO_1_PIN = 9;                                       // Claw grab servo
const int SERVO_2_PIN = 8;                                       // Claw lift servo
const int BUTTON_PIN = 11;                                       // Pin for button to start the program (not stop if pressed again!)

// --------- Sensor Variables ---------                          // --------- Sensor Variables ---------
bool line_1, line_2, line_3;                                     // Booleans to describe if each line sensor has a line below or not
int IR_amplitude;                                                // Output magnitude of IR sensor (for locating dummies)
int max_IR_amplitude;                                            // Maximum value of IR recorded throughout a rotation
int US_amplitude;                                                // Distance recorded by the ultrasonic sensor in cm. Return 200 if out of range (dist > 200cm)


// --------- Logic Variables ---------                           // --------- Logic Variables ---------
bool finished_dropping = false;                                  // Bool to track drop_off_dummy() function return state to track when dummy has been dropped off yet
bool picked_up_yet = false;                                      // Bool to track pick_up_dummy() function return state to track when dummy has been picked up yet
int what_dummy_am_I;                                             // The dummy that is detected (0 for line, 1 for red box, 2 for blue box)
bool dummy_located = false;                                      // If a dummy has been spotted using IR amplitude or not
int number_dummies_saved = 0;                                    // Number of dummies returned to a box
int hit_cross_roads = 0;                                         // Used to count if a cross-roads has been crossed yet. Used in returning to home to stop after second cross-roads

bool currently_adjusting = false;                                // Bool used by home_dummy() to track when the robot is currently doing an adjustment sweep
bool finished_adjusting = false;                                 // Bool used by home_dummy() to track when the robot has finished doing an adjustment sweep

float percentage_of_sweep_completed = 0;                         // Used by point_towards_nearest_dummy to track if the robot finds a dummy to its left or right by timing how long it sweeps for before locking-on

bool dummy_side_right = false;                                   // False = Left, True = Right. Keeps track of which side the located dummy is on

bool follow_line_output = false;                                 // Bool to store the value of follow_line() to not require it to be run multiple times per tick

bool first_dummy_delivered = false;                              // Bool to track if the dummy in the white box has been returned or not yet. Used to know what to look for when finding the end white box again


// --------- Timer/Timing Variables ---------                    // --------- Timer/Timing Variables ---------
unsigned long delay_5s_start_time = 0;                           // Used to pause for 5s before grabbing each dummy
unsigned long reverse_2s = 0;                                    // Used to reverse for 2 seconds after hitting the white box to return to the center of the search area
unsigned long drop_off_reverse = 0;                              // Used to reverse for 1 second after dropping off dummy in red/blue box
unsigned long drive_1s_timer = 0;                                // Used to drive 1 second more after returning to line after grabbing dummy from middle of search area
unsigned long drive_home_timer = 0;                              // Used to drive 2.5 seconds after reaching the final box to stop within it
unsigned long sweep_start_time = 0;                              // Timer used to time how long to sweep when searching for a dummy with IR amplitude sensor

unsigned long home_sweep_time = 0;                               // Timer used to drive forwards for a fixed amount of time before each adjustment in home_dummy()
unsigned long creep_time = 3000;                                 // Time that home_dummy() waits between each adjustment

unsigned long turn_timer = 0;                                    // Timer used for turning 360, 180 or 90 degrees

const int full_360_time = 10000.0;                                // Time taken for full 360 at set motor speed
const int motor_360_speed = 255;                                 // Motor speed used for turning a full 360

unsigned long pick_up_dummy_start_time = 0;                      // Time to start picking up dummy
unsigned long drop_off_dummy_start_time = 0;                     // Time to start dropping off dummy

unsigned long avoid_first_sweep_time = 0;

unsigned long turn_onto_line_timer = 0;


unsigned long reverse_again_timer = 0;

unsigned long drive_a_little_forwards_timer = 0;

bool temp_test_var1 = false;
int temp_test_var2 = 0;
bool temp_test_var3 = false;
bool temp_test_var4 = false;
bool temp_test_var5 = false;
int temp_test_var6 = 0;
int temp_test_var7 = 0;
int temp_test_var8 = 0;

int five_seconds = 3000;                                         //depsite the variable name used for fine tuning hard coded times

int proximity_counter = 0;                                       // Proximity counters are used to ignore glitch values from the US sensor (i.e. require 3 values below/above threshold in a row to confirm)
int proximity_counter2 = 0;
int proximity_counter3 = 0;
int proximity_counter4 = 0;


// --------- Angle Variables ---------                           // --------- Angle Variables ---------

int angle1 = 195;                                                // used for fine tuning angles, easier for global variables than going through the code to find each use case
int angle2 = 90;
int angle3 = 90;
int angle4 = 85;
int angle5 = 90;
int angle6 = 190;
int angle7 = 170;


// --------- Motor Initialisation ---------                      // --------- Motor Initialisation ---------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();              // Create motorshield object called 'AFMS' with default I2C address. Put I2C address in brackets if different address needed
Adafruit_DCMotor *left_motor = AFMS.getMotor(LM_port);           // Initialise left  motor using assigned motor shield port
Adafruit_DCMotor *right_motor = AFMS.getMotor(RM_port);          // Initialise right motor using assigned motor shield port

Servo claw_servo;                                                // Servo for grabbing dummy
Servo lift_servo;                                                // Servo for lifting claw


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
      digitalWrite(LED1_PIN,HIGH);                               // Write HIGH output to LED Pin 1 (orange LED)
    }
    else if (tick_counter*tick_length % 500 > 249) {             // If in second half of 500ms time period (since 50% duty cycle)
      digitalWrite(LED1_PIN,LOW);                                // Write LOW output to LED Pin 1 (orange LED)
    }
  }
  else {
    digitalWrite(LED1_PIN,LOW);                                  // Turn orange LED off if not moving (writing spd = 0)
  } 
}

                                                                 // Function to record voltage amplitude of IR sensor 10% duty cycle signal
                                                                 // IR_PIN must be an analog pin for any non-binary results.
                                                                 // Wave is ~6ms long, and samples take ~ 112us, so set samples to AT LEAST >54 to ensure peak is included in measurement (preferably > 108)
int measure_IR_amplitude(int samples = 107) {                    // CODE IS BLOCKING TO ALLOW FAST SAMPLING RATE - blocking time will increase linearly with samples
  
  int max_val = 0;                                               // Variable to store max. voltage measured
  int new_val;                                                   // Variable to store current measurement
  for (int i=0; i<samples; i++) {
    new_val = analogRead(IR_AMP_PIN);                            // Measure current voltage from photodiode
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

  if (analogRead(LS1_PIN) < 150) {                               // Checks if we meet the cutoff or not (less than 500 = White)
    line_1 = true;
  }
  else {
    line_1 = false;
  }
  if (analogRead(LS2_PIN) < 800) {                               // Repeat for other sensors
    line_2 = true;
  }
  else {
    line_2 = false;
  }
  if (analogRead(LS3_PIN) < 750) {
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
    drive_motor(left_motor, 255, false);
    drive_motor(right_motor, 150, false);
  }                                                              
  else if ((not line_1) and (line_3)){                           // Hit line on RHS  so steering LEFT
    drive_motor(left_motor, 150, false);
    drive_motor(right_motor, 255, false);
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
bool point_towards_nearest_dummy(int sweep_time = full_360_time, float threshold = 0.7, int motor_sweep_speed = 255) {

  percentage_of_sweep_completed = 0.0;

  if (sweep_start_time == 0 and not dummy_located) {             // If haven't started sweeping yet (first time activating function) and haven't already found dummy
    sweep_start_time = tick_counter*tick_length;                 // Set start time to current time
    max_IR_amplitude = 0;                                        // Initialise IR amplitude at 0
    drive_motor(left_motor, 0, false);                           // Stop driving to prepare for turning around
    drive_motor(right_motor, 0, false);
  }

  if (tick_counter*tick_length < sweep_start_time+sweep_time) {  // If still within first sweep (sweep_start_time -> sweep_start_time + sweep_time)
    
    drive_motor(left_motor, motor_sweep_speed, true);            // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
    drive_motor(right_motor, motor_sweep_speed, false);
  
    IR_amplitude = measure_IR_amplitude(107);                    // Take IR sensor input
  
    if (IR_amplitude > max_IR_amplitude) {                       // If new max found...
      max_IR_amplitude = IR_amplitude;                           // ...set max to current value
    }
  }                                                              // If outside of first sweep and within second. The second sweep lasts no more than 3x as long as the first sweep
  else if (tick_counter*tick_length < sweep_start_time + 4*sweep_time) {

    if (not dummy_located) {                                     // If haven't found dummy yet

      IR_amplitude = measure_IR_amplitude(107);                  // Read IR amplitude

      if (IR_amplitude < max_IR_amplitude*threshold) {           // If IR value hasn't reached cutoff (threshold*max_IR_amplitude), then continue turning, searching

        drive_motor(left_motor, motor_sweep_speed, false);       // Drive left in reverse, right forwards to turn counter-clockwise (turn slowly)
        drive_motor(right_motor, motor_sweep_speed, true);
        
      }
      else {                                                     // If reached cutoff (threshold*max_IR_amplitude), then declare dummy found and stop turning, reset sweep_start_time
        percentage_of_sweep_completed = (tick_counter*tick_length - sweep_start_time - sweep_time) / float(sweep_time);
        
        dummy_located = true;                                    // Dummy found
        sweep_start_time = 0;                                    // Reset sweep start time
        drive_motor(left_motor, 0, false);                       // Stop driving
        drive_motor(right_motor, 0, false);                      // Stop driving
        
        return true;
        
      }
    }                                                            // If outside 2nd sweep and no dummy found, just stop (and have a little cry)
    else if (tick_counter*tick_length > sweep_start_time + 4*sweep_time){
      drive_motor(left_motor, 0, false);                         // Stop driving
      drive_motor(right_motor, 0, false);                        // Stop driving
    }
  }
  return false;
}

bool home_dummy() {                                              // used for finding a dummy that is off the line by accuratinly honing in on the ir signal

  US_amplitude = take_ultrasonic_reading()

  if (US_amplitude < stopping_distance) {                        // Use the US to detect when close to dummy
    proximity_counter3++;
  }
  else {
    proximity_counter3 = 0;
  }
  if (proximity_counter3 > 5) {                                  // uses the average of 5 readings in a row to confirm within 5cm as US sometimes unreliable
    drive_motor(left_motor,0,false);
    drive_motor(right_motor,0,false);
    return true;
  }
  if (US_amplitude < 1.5*stopping_distance) {                    // slows down as we approach so as not to go clattering into dummy
    proximity_counter4++; 
  }
  else {
    proximity_counter4 = 0;
  }
  if (proximity_counter4 > 5) {
    drive_motor(left_motor,150,false);
    drive_motor(right_motor,150,false);
  }
  else { 
    if (home_sweep_time == 0) {
      home_sweep_time = tick_counter*tick_length;
    }
    else if (tick_counter*tick_length < home_sweep_time + creep_time) {
      drive_motor(left_motor,255,false);
      drive_motor(right_motor,255,false);
    }                                                            // spins for a second when closer so we can be sure we have found a dummy
    else if (tick_counter*tick_length < home_sweep_time + creep_time + 1000) {
      drive_motor(left_motor,255,false);
      drive_motor(right_motor,255,true);
    }
    else {
      dummy_located = false;
      if (point_towards_nearest_dummy(2000,0.95,255)) {
        home_sweep_time = 0;
      }
    }
  }
  return false;
  
}

bool turn(int angle, bool clockwise = true) {                    // Turns a set angle. Should be accurate since uses carefully calibrated motor speed + turn time. Returns true when done
  if (turn_timer == 0) {                                         // If haven't started turning yet
    turn_timer = tick_counter*tick_length;                       // Set current time to turn start time
    return false;
    
  }
  else if (tick_counter*tick_length < turn_timer + (angle/360.0)*full_360_time) {
    drive_motor(left_motor, motor_360_speed, not clockwise);     // Drive for the correct proportion of a 360 turn at the calibrated speed
    drive_motor(right_motor, motor_360_speed, clockwise);
    return false;                                                // Not done yet so return false
  }
  else if (tick_counter*tick_length > -1 + turn_timer + (angle/360.0)*full_360_time) {
    drive_motor(left_motor, 0, false);                           // Stop motors
    drive_motor(right_motor, 0, false);
    turn_timer = 0;
    return true;                                                 // Done so return true
  }
}

bool turn_onto_line(bool turn_right = true){                     // gets the robot back onto the line after having picked up dummy
  read_line_sensors();
  if (not (line_1 or line_2 or line_3) and turn_onto_line_timer == 0) {
    drive_motor(left_motor,255,false);                           // drives on if not found line
    drive_motor(right_motor,255,false);
  }
  else if (line_1 or line_2 or line_3 or turn_onto_line_timer != 0) {
    if (turn_onto_line_timer == 0) {
      turn_onto_line_timer = tick_counter*tick_length;
    }
    else if (tick_counter*tick_length < turn_onto_line_timer + 1500) {
      drive_motor(left_motor,255,false);                         // once found line keep going for 1.5s 
      drive_motor(right_motor,255,false);
    }
    else if (tick_counter*tick_length > turn_onto_line_timer + 1499) {
      if (not line_2) {                                          // then turn so on line and facing along the line
        drive_motor(left_motor,255,turn_right);
        drive_motor(right_motor,255,not turn_right);
      }
      else {
        turn_onto_line_timer = 0;
        return true;                                             // Done so return true
      }
    }
  }
  return false;
}

int identify_dummy(){                                            // Reads IR input signal and determines which dummy is in front of it 
                                                                 // Function to return modulation type as enumerated integer (1=mod, 2=mix, 3=unmod, 4=bad)
  
  // Number of times to sample the IR reading (1 sample ~=~ 3.42us)
  int samples = 3509;                                            // Changing this will change the boundary conditions! (averages get skewed!)
  float lower_mod = 0.85;                                        // Lower boundary of average value for modulated signal
  float lower_mix = 0.91;                                        // Lower boundary of average value for mixmodulated signal (and upper for modulated)
  float lower_unm = 0.97;                                        // Lower boundary of average value for unmodulated signal (and upper for mixmodulated)
  
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum = sum + (int) digitalRead(IR_MOD_PIN);                   // Continually sample value of wave, and add up total
  }
  float total = (float(sum)/float(samples));                     // Calculate mean value of wave
  
  //Serial.print(total);
  //Serial.print(" ");
  if (total >= lower_mod and total < lower_mix) {
    digitalWrite(LED2_PIN,HIGH);
    digitalWrite(LED3_PIN,HIGH);
    return 1;                                                    // Modulated, white box
  } 
  else if (total >= lower_mix and total < lower_unm) {
    digitalWrite(LED2_PIN,HIGH);
    digitalWrite(LED3_PIN,LOW);
    return 2;                                                    // Mixmodulated, blue box (left hand turn)
  }
  else if (total >= lower_unm) {
    digitalWrite(LED2_PIN,LOW);
    digitalWrite(LED3_PIN,HIGH);
    return 3;                                                    // Unmodulated, red box (right hand turn)
  }
  else {
    digitalWrite(LED2_PIN,LOW);
    digitalWrite(LED3_PIN,LOW);
    return 0;                                                    // Something weird???
  }
}

bool pick_up_dummy(){                                            // Drive servos to pick up dummy. Return true when finished
  if (pick_up_dummy_start_time == 0) {
    pick_up_dummy_start_time = tick_counter*tick_length;         // Initialise
  }                                                              // Grab (1s delay)
  else if (tick_counter*tick_length < pick_up_dummy_start_time + 1000) {
    claw_servo.write(100);
    lift_servo.write(123);
  }                                                              // Lift (1s delay)
  else if (tick_counter*tick_length < pick_up_dummy_start_time + 2000) {
    claw_servo.write(100);
    lift_servo.write(85);
  }
  else {
    pick_up_dummy_start_time = 0;
    return true; 
  }
  return false;
}

bool drop_off_dummy(){                                           // Drive servos to released dummy. Return true when finished
  if (drop_off_dummy_start_time == 0) {
    drop_off_dummy_start_time = tick_counter*tick_length;        // Initialise
    claw_servo.write(100);
    lift_servo.write(85);
  }                                                              // lower claw
  else if (tick_counter*tick_length < drop_off_dummy_start_time + 1000) {
    claw_servo.write(100);
    lift_servo.write(120);
  }                                                              // release
  else if (tick_counter*tick_length < drop_off_dummy_start_time + 2000) {
    claw_servo.write(20);
    lift_servo.write(120);
  }                                                              // raise claw (all delayed a little)
  else if (tick_counter*tick_length < drop_off_dummy_start_time + 2500) {
    claw_servo.write(20);
    lift_servo.write(123);
    drop_off_dummy_start_time = 0;
    return true;
  }
  return false;
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
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LS1_PIN, INPUT);                                       // Declare the line sensor pins as inputs
  pinMode(LS2_PIN, INPUT);
  pinMode(LS3_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);                                     // Use TRIG pin as output for ultrasonic sensor (this pin gives an ultrasonic pulse)
  pinMode(ECHO_PIN, INPUT);                                      // Use ECHO pin as input for ultrasonic sensor (this pin reads the echoed ultrasonic pulse from the TRIG pin)
  pinMode(IR_AMP_PIN,INPUT);                                     // Pin for measuring the amplitude of the IR sensor for use in locating dummies
  pinMode(IR_MOD_PIN, INPUT);                                    // Pin for measuring modulated signal to detect dummies
  pinMode(BUTTON_PIN, INPUT);
  pinMode(SERVO_1_PIN, OUTPUT);
  pinMode(SERVO_2_PIN, OUTPUT);
  
  robot_state = 0;                                               // Reset robot state to default

  left_motor->setSpeed(0);                                       // Initialise left motor
  left_motor->run(FORWARD);
  left_motor->run(RELEASE); 

  right_motor->setSpeed(0);                                      // Initialise right motor
  right_motor->run(FORWARD);
  right_motor->run(RELEASE); 

  claw_servo.attach(SERVO_1_PIN);
  lift_servo.attach(SERVO_2_PIN);

  claw_servo.write(20);
  lift_servo.write(120);
}

// --------- Main Loop ---------                                 // --------- Main Loop ---------
void loop() {                                                    // Function that runs repeatedly whilst the robot is on

  if (digitalRead(BUTTON_PIN) == 1) {                            // If button pressed to start the program
    program_started = true;                                      // Start program
  }
  
  if (program_started) {
    if (not using_servos) {
      claw_servo.write(20);
      lift_servo.write(120);
    }
    if (robot_test_state == -1) {                                // Test -1: Display all sensor/input values
      Serial.print("IR Amp: ");
      Serial.print(measure_IR_amplitude(107));                   // IR Amplitude
      Serial.print(" / 1023, US Dist: ");
      Serial.print(take_ultrasonic_reading());                   // Ultrasonic distance
      read_line_sensors();                                       // Update line sensors
      Serial.print(" cm, LS1: ");
      Serial.print(line_1);                                      // Print each line sensor boolean (true = on line, false = off line)
      Serial.print(",");
      Serial.print(analogRead(LS1_PIN));
      Serial.print(", LS2: ");
      Serial.print(line_2);
      Serial.print(",");
      Serial.print(analogRead(LS2_PIN));
      Serial.print(", LS3: ");
      Serial.print(line_3);
      Serial.print(",");
      Serial.print(analogRead(LS3_PIN));
      Serial.print(", IR MOD: ");
      Serial.println(identify_dummy());
    }
    else if (robot_test_state == 1) {                            // Test 1: Spin the wheels without control of any kind
      if (tick_counter*tick_length > 3000) {                     // Wait [3] seconds before beginning test
        drive_motor(left_motor,255,false);                       // Use the electrical function to spin each motor at max speed
        drive_motor(right_motor,255,false);
      }
    }
    else if (robot_test_state == 2 or robot_test_state == 3) {   // Test 2/3: Drive forwards for a fixed time (to give ~1m forwards)
      if (tick_counter*tick_length > 7000) {                     // Drive for [4] seconds
        drive_motor(left_motor,0,false);                         // Stop motors after test complete
        drive_motor(right_motor,0,false);
      }
      else if (tick_counter*tick_length > 3000) {                // Wait [3] seconds before beginning test
        drive_motor(left_motor,255,false);                       // Drive both motors forwards at full speed
        drive_motor(right_motor,255,false);
      }
    }
    else if (robot_test_state == 4) {                            // Test 4: 360 turn in each direction, then reverse for ~1m
      if (tick_counter*tick_length > 13000) {                    // Stop motors after test complete
        drive_motor(left_motor,0,true);                                    
        drive_motor(right_motor,0,true);
      }
      else if (tick_counter*tick_length > 9000) {                // Reverse for [4] seconds = ~1m
        drive_motor(left_motor,255,true);                                    
        drive_motor(right_motor,255,true);
      }
      else if (tick_counter*tick_length > 6000) {                // Spin Counter-clockwise, slowing down to 0 at the end of the 3-second turn
        drive_motor(left_motor,255-255*(tick_counter*tick_length-6000)/3000,true);                                  
        drive_motor(right_motor,255-255*(tick_counter*tick_length-6000)/3000,false);
      }
      else if (tick_counter*tick_length > 3000) {                // Wait [3] seconds before beginning test
                                                                 // Spin Clockwise, slowing down to 0 at the end of the 3-second turn, we used proportional control in this test but found it unnecesary for the main algorithm
        drive_motor(left_motor,255-255*(tick_counter*tick_length-3000)/3000,false);                                  
        drive_motor(right_motor,255-255*(tick_counter*tick_length-3000)/3000,true);
      }
    }
    else if (robot_test_state == 5) {                            // Test 5: Crash into dummy. Point at nearest dummy using IR and drive forwards. don't. stop.
      if (tick_counter*tick_length > 3000) {
        if (not temp_test_var1) {                                // Use point_towards_nearest_dummy() until dummy_located
          temp_test_var1 = point_towards_nearest_dummy(2000,0.95,255);
        }
        else {
          drive_motor(left_motor,255,false);                     // Drive!
          drive_motor(right_motor,255,false);
        }
      }
    }
    else if (robot_test_state == 9) {                            // Test 9: Use claw to grab, lift, drive, lower, release, reverse (just test servos a lot)
  
      if (tick_counter*tick_length < 3000) {                     // Default state
        claw_servo.write(20);
        lift_servo.write(118);
      }
      else if (tick_counter*tick_length < 4000) {                // Grab
        claw_servo.write(60);
        lift_servo.write(118);
      }
      else if (tick_counter*tick_length < 5000) {                // Lift
        claw_servo.write(60);
        lift_servo.write(70);
      }
      else if (tick_counter*tick_length < 17000) {               // Drive
        claw_servo.write(60);
        lift_servo.write(70);
        drive_motor(right_motor,255,false);
        drive_motor(left_motor,255,false);
      }
      else if (tick_counter*tick_length < 18000) {               // Stop
        claw_servo.write(60);
        lift_servo.write(70);
        drive_motor(right_motor,0,false);
        drive_motor(left_motor,0,false);
      }
      else if (tick_counter*tick_length < 18500) {               // Lower
        claw_servo.write(60);
        lift_servo.write(115);
      }
      else if (tick_counter*tick_length < 19000) {               // Release
        claw_servo.write(20);
        lift_servo.write(118);
      }
      else if (tick_counter*tick_length < 21000) {               // Return to normal state
        claw_servo.write(20);
        lift_servo.write(118);
      }
      else if (tick_counter*tick_length < 22000) {               // Reverse away
        claw_servo.write(20);
        lift_servo.write(120);
        drive_motor(right_motor,255,true);
        drive_motor(left_motor,255,true);
      }
      else if (tick_counter*tick_length < 23000) {               // Stop
        claw_servo.write(20);
        lift_servo.write(120);
        drive_motor(right_motor,0,true);
        drive_motor(left_motor,0,true);
      }
    }
    
    else if (robot_test_state == 12) {                           // Test 12: Follow line
      if (tick_counter*tick_length > 3000) {                     // Wait [3] seconds before beginning test
        follow_line();                                           // Just run the follow_line() function (forwards)
      }
    }
    else if (robot_test_state == 13) {                           // Test 13: Drive until obstacle is 20cm away, then stop. Then turn until there is nothing for 100cm, then continue drive
      bool temp_test_var = false;
      if (tick_counter*tick_length > 3000) {                     // Wait [3] seconds before beginning test
        if (not temp_test_var) {                                 // Driving forwards mode
          if (take_ultrasonic_reading() > 20) {                  // If not 20cm away yet
            drive_motor(left_motor,255,false);
            drive_motor(right_motor,255,false);
          }
          else {
            temp_test_var = true;                                // Switch mode
          }
        }
        if (temp_test_var) {                                     // Turning mode
          if (take_ultrasonic_reading() < 100) {                 // If wall still closer than 100cm away
            drive_motor(left_motor,255,false);
            drive_motor(right_motor,255,true);
          }
          else {
            temp_test_var = false;                               // Switch mode
          }
        }
      }
    }
    else if (robot_test_state == 14) {                           // Test 14: Locate and home dummy, then grab it.
      if (tick_counter*tick_length > 3000) {                     // Wait 3 seconds
        if (not temp_test_var3 and temp_test_var2 == 0) {        // Point towards dummy, use standard settings
          temp_test_var3 = point_towards_nearest_dummy(10000,0.8,255);            
          using_servos = false;
        }
        else {
          temp_test_var2 = 1;                                    // Located!
        }
        if (temp_test_var2 == 1) {                               // Home_dummy until stopped right in front of it
          if (home_dummy()) {
            temp_test_var2 = 2;                                  // Switch mode again
            using_servos = false;
          }
        }
        if (temp_test_var2 == 2) {                               // Use claw to grab dummy
          using_servos = false;
          drive_motor(left_motor,0,false);
          drive_motor(right_motor,0,false);
        }
      }
    }
    else if (robot_test_state == 15) {                           // Test 15: Identify dummy. Whilst in front of dummy, blink the correct LEDs to show which dummy this one is       
      if (tick_counter*tick_length > 3000) {                     // Wait 3 seconds
        identify_dummy();                                        // Identify dummy testing
      }
    }
    else if (robot_test_state == 17) {                           // Test 17: Turn 180 degrees then stop and turn the red LED on
      if (tick_counter*tick_length > 500) {                      // Wait 0.5 seconds
        if (not temp_test_var4) {                                // If not stopped yet
          temp_test_var4 = turn(angle7,false);                   // Turn
        }
        else {
          digitalWrite(LED3_PIN,HIGH);                           // Turn red LED on
        }
      }
    }
    else if (robot_test_state == 21) {                           // this corresponds to robot state 0 of main code
      if (take_ultrasonic_reading() > 5){                        // this should drive us up over the ramp to the first dummy
        follow_line();
      }
      else {
        drive_motor(left_motor,0,false);                         // stop if within 5cm
        drive_motor(right_motor,0,false);
        if (delay_5s_start_time == 0){
          delay_5s_start_time = tick_counter*tick_length;
        }
        if (((tick_counter * tick_counter) > (0+delay_5s_start_time)) and ((tick_counter * tick_length) < (delay_5s_start_time + 5000))){
          what_dummy_am_I = identify_dummy();                    // identify dummy
        }
        if ((tick_length * tick_counter) > (delay_5s_start_time + 5000)){
          picked_up_yet = pick_up_dummy();                       // pick up dummy
        }
        if (picked_up_yet){
          delay_5s_start_time = 0;
          robot_state = 1;
          picked_up_yet = false;
        }
      }
    }
    else if (robot_test_state == 22) {                           // Test 22: Test used to calibrate turning. Turns 2 full turns
      if (not temp_test_var5) {
        temp_test_var5 = turn(720);
      }
    }
    else if (robot_test_state == 23) {                           // Test 23: Accurate turning demo
      if (tick_counter*tick_length < 3000) {
        temp_test_var6 = 0;
      }
      else {
        if (temp_test_var6 == 0) {
          if (turn(360,false)) {
            temp_test_var6 = 1;
          }
        }
        else if (temp_test_var6 == 1) {
          if (turn(360,true)) {
            temp_test_var6 = 2;
          }
        }
        else if (temp_test_var6 == 2) {
          if (turn(90,false)) {
            temp_test_var6 = 3;
          }
        }
        else if (temp_test_var6 == 3) {
          if (turn(90,true)) {
            temp_test_var6 = 4;
          }
        }
        else if (temp_test_var6 == 4) {
          digitalWrite(LED3_PIN,HIGH);
        }
      }
    }
    else if (robot_test_state == 24) {                           // tests home_dummy() algorithm and then stops and waits
      using_servos = false;
      if (home_dummy()) {
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        delay(100000);
      }
    }
    else if (robot_test_state == 25) {                           // tests pick_up_dummy() algorithm followed by drop off dummy algorithm
      using_servos = true;
      if (temp_test_var7 == 0) {
        if (pick_up_dummy()) {
          temp_test_var7 = 1;
        }
      }
      else if (temp_test_var7 == 1){
        if (drop_off_dummy()) {
          temp_test_var7 = 2;
        }
      }
    }
    else if (robot_test_state == 26) {                           // more servo testing also printing ultrasonic
      using_servos = true;
      if (tick_counter*tick_length < 3000) {
        claw_servo.write(20);
        lift_servo.write(120);
      }
      else if (tick_counter*tick_length < 4000) {
        claw_servo.write(75);
        lift_servo.write(120);
      }
      else if (tick_counter*tick_length > 4000) {
        claw_servo.write(75);
        lift_servo.write(70);
        Serial.println(take_ultrasonic_reading());
      }
    }
    else if (robot_test_state == 100) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 0;
        
      }
      else if (not main_test_started and robot_state == 0) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (robot_state != 0) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
    else if (robot_test_state == 110) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 1;
        what_dummy_am_I = 1;
        robot_sub_state = 0;
        
      }
      else if (not main_test_started and robot_state == 1) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (robot_state != 1) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
    else if (robot_test_state == 111) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 1;
        what_dummy_am_I = 2;
        robot_sub_state = 1;
        number_dummies_saved = 0;
        using_servos = true;
        claw_servo.write(75);
        lift_servo.write(70);
        
      }
      else if (not main_test_started and robot_state == 1) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (robot_state == 4) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
    else if (robot_test_state == 112) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 1;
        what_dummy_am_I = 2;
        robot_sub_state = 4;
        number_dummies_saved = 0;
        
      }
      else if (not main_test_started and robot_state == 1) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (robot_state != 1) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
    else if (robot_test_state == 120) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 2;
        robot_sub_state = 1;
        number_dummies_saved = 1;
        
      }
      else if (not main_test_started and robot_state == 2) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (false) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
    else if (robot_test_state == 130) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 3;
        robot_sub_state = 0;
        number_dummies_saved = 1;
        dummy_side_right = false;
        
      }
      else if (not main_test_started and robot_state == 3) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (robot_state != 3) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
    else if (robot_test_state == 140) {                          // Tests 100+: Running parts of the main code. The program sets all the correct state variables and leaves the testing state to run the main code                          
      if (tick_counter*tick_length < 3000) {
        if (tick_counter*tick_length % 500 < 250) {
          digitalWrite(LED3_PIN,HIGH);
        }
        else {
          digitalWrite(LED3_PIN,LOW);
        }

        robot_state = 4;
        robot_sub_state = 0;
        number_dummies_saved = 3;
        
      }
      else if (not main_test_started and robot_state == 4) {
        digitalWrite(LED3_PIN,LOW);
        main_test_started = true;
      }
      else if (robot_state != 4) {
        main_test_started = false;
        drive_motor(left_motor,0,false);
        drive_motor(right_motor,0,false);
        digitalWrite(LED3_PIN,HIGH);
      }
    }
      

// ------------------------------------------------------------ MAIN PROGRAM ------------------------------------------------------------
    if (robot_test_state == 0 or (robot_test_state >= 100 and main_test_started)) {
      if (robot_state == 0) {
        int temp_ultrasonic = 0;
        temp_ultrasonic = take_ultrasonic_reading();             // this should drive us up over the ramp to the first dummy and stops in 5cm
        if (temp_ultrasonic > stopping_distance and delay_5s_start_time == 0){
          follow_line();
          proximity_counter = 0;
          digitalWrite(LED2_PIN,LOW);
          digitalWrite(LED3_PIN,LOW);
        }
        else{
          proximity_counter ++;                                  // proximity counter is used to ensure repeated readings of <5cm instead of 1 as that could be a glitch
        }
        if (proximity_counter > 3) {
          drive_motor(left_motor,0,false);                                    
          drive_motor(right_motor,0,false);
          if (delay_5s_start_time == 0){
            delay_5s_start_time = tick_counter*tick_length;
            using_servos = false;
          }
          else if (((tick_counter * tick_counter) > (-1+delay_5s_start_time)) and ((tick_counter * tick_length) < (delay_5s_start_time + five_seconds))){
            what_dummy_am_I = identify_dummy();                  // identifies dummy using LEDs for 5s, to avoid miss identification use a modal average of what idenify dummy returns
            if (what_dummy_am_I == 1) {
              readings_mod++;
            }
            else if (what_dummy_am_I == 2) {
              readings_mix++;
            }
            else if (what_dummy_am_I == 3) {
              readings_unm++;
            }
            using_servos = false;
          }
          else if ((tick_length * tick_counter > delay_5s_start_time + five_seconds - 1) and not dummy_identity_decided) {
            if (readings_mod > readings_mix and readings_mod > readings_unm) {
              what_dummy_am_I = 1;                               // displays the correct LEDs for the given dummy
              dummy_identity_decided = true;
              digitalWrite(LED2_PIN,HIGH);
              digitalWrite(LED3_PIN,HIGH);
            }
            else if (readings_mix > readings_mod and readings_mix > readings_unm) {
              what_dummy_am_I = 2;
              dummy_identity_decided = true;
              digitalWrite(LED2_PIN,HIGH);
              digitalWrite(LED3_PIN,LOW);
            }
            else if (readings_unm > readings_mix and readings_unm > readings_mod) {
              what_dummy_am_I = 3;
              dummy_identity_decided = true;
              digitalWrite(LED2_PIN,LOW);
              digitalWrite(LED3_PIN,HIGH);
            }
            readings_mod = 0;                                    // reset varaibles when dummy_identity_detected is true
            readings_mix = 0;
            readings_unm = 0;
          }
          else if ((tick_length * tick_counter) > (delay_5s_start_time + five_seconds - 1) and not picked_up_yet and dummy_identity_decided){
            using_servos = true;
            picked_up_yet = pick_up_dummy();                     // calls the pick_up_dummy() function 
          }
          else if (picked_up_yet){                               // when done picking up move on and reset variables
            delay_5s_start_time = 0;
            robot_state = 1;
            picked_up_yet = false;
            dummy_identity_decided = false;
          }
        }
      }

      if ((what_dummy_am_I == 1) and robot_state == 1){          // modulated dummy going to white box from on line position, at the end of this IF statement the robot is on line facing dummy in white box
        if (robot_sub_state == 0){
          if (follow_line()) {                                   // when it gets to cross on top of box it calls next state
            robot_sub_state = 1;
          }
        }
        if (robot_sub_state == 1) {                              // stops, drops off dummy, sets LEDs and resets variables
          finished_dropping = drop_off_dummy();
          drive_motor(right_motor,0,false);
          drive_motor(left_motor,0,false);
          using_servos = true;
          if (finished_dropping) {
            number_dummies_saved += 1;
            robot_sub_state = 2;
            using_servos = false;
            first_dummy_delivered = true;
            digitalWrite(LED1_PIN,LOW);
            digitalWrite(LED2_PIN,LOW);
            digitalWrite(LED3_PIN,LOW);
          }
        }
        else if (robot_sub_state == 2){
          if (reverse_2s == 0){
            reverse_2s = tick_counter * tick_length;
          }                                                      //reverse for 2s so when turns doesn't hit the dummy
          else if (tick_counter * tick_length < reverse_2s + 2000){
            drive_motor(left_motor,255,true);
            drive_motor(right_motor,255,true);
          }
          else{                                                  // reset variables and call next state
            robot_state = 2;
            robot_sub_state = 0;
            reverse_2s = 0;
            finished_dropping = false;
            drive_motor(left_motor,0,false);
            drive_motor(right_motor,0,false);
          }
        }
      }
      else if ((what_dummy_am_I == 2 or what_dummy_am_I == 3) and (robot_state == 1)) {
        // delivers first dummy that's on the line to the RED box  modulated ir signal, starts with dummy in grasp
        if (robot_sub_state == 0){                               // turns to face home
          if (turn(angle1)){
            robot_sub_state = 1;
          }
        }
        else if (robot_sub_state == 1){
          if (follow_line()){                                    //drive back to start untill encounter cross roads
            robot_sub_state = 2;
            drive_1s_timer = 0;
          }
        }
        else if (robot_sub_state == 2){
          if (drive_1s_timer == 0){
            drive_1s_timer = tick_counter * tick_length; 
          } 
                                                                 //drive 1s forward after first finding horizontal line so that when it turns 90 it will drop into box
          else if (tick_length * tick_counter < drive_1s_timer + 1000){
            follow_line();
          }
          else{
            drive_motor(left_motor, 0, false);
            drive_motor(left_motor, 0, false);
            robot_sub_state = 3;
          }
           
        }
        else if (robot_sub_state == 3){
          if (what_dummy_am_I == 2){
            if(turn(angle2, true)){                              //turn 90 clockwise
              robot_sub_state = 4;
            }
          }
          else if (what_dummy_am_I == 3){
            if(turn(angle3, false)){                             //turn 90 anticlockwise
              robot_sub_state = 4;
            }
          }
        }
        else if (robot_sub_state == 4) {
          if (drive_a_little_forwards_timer == 0) {              // drive into box a little bit so can back out onto line
            drive_a_little_forwards_timer = tick_counter*tick_length;
          }
          else if (tick_counter*tick_length < drive_a_little_forwards_timer + 1000) {
            drive_motor(left_motor,255,false);
            drive_motor(right_motor,255,false);
          }
          else {
            drive_motor(left_motor,0,false);
            drive_motor(right_motor,0,false);
            drive_a_little_forwards_timer = 0;
            robot_sub_state = 5;
          }
        }
        else if (robot_sub_state == 5){
          finished_dropping = drop_off_dummy();                  // drop off dummy, call next state and reset variables
          if (finished_dropping){                   
            number_dummies_saved += 1;
            robot_sub_state = 6;
            using_servos = false;
            digitalWrite(LED1_PIN,LOW);
            digitalWrite(LED2_PIN,LOW);
            digitalWrite(LED3_PIN,LOW);
           }
         }
        
        else if (robot_sub_state == 6){
          if (drop_off_reverse == 0){
            drop_off_reverse = tick_counter * tick_length;
          }                                                      //reverse for 1s so can return to line
          else if (tick_counter * tick_length < drop_off_reverse + 1000){
            drive_motor(left_motor,255,true);
            drive_motor(right_motor,255,true);
          }
          else{
            robot_sub_state = 7;
            drop_off_reverse = 0;
          }
        }

        else if (robot_sub_state == 7){                          // turns the robot back onto line facing danger area
          if (what_dummy_am_I == 2){
            if(turn(angle4, true)){
              robot_sub_state = 8;
              turn_timer = 0;
            }
          }
          else {
            if(turn(angle5, false)){
              robot_sub_state = 8;
              turn_timer = 0;
            }
          }
        }
        else if (robot_sub_state == 8){
          int temp_ultrasonic = 0;
          temp_ultrasonic = take_ultrasonic_reading();
          if (temp_ultrasonic > 14){                             // this should drive us up over the ramp and stop 14cm from end so we can sweep for dummies again.
                                                                 // requires 3 US values under 14cm in a row to activate to avoid any random disturbance
            if (follow_line()) {
              proximity_counter2 = 100;                          // if US doesn't work then uses horizontal line to of white box to stop, this mostly depends if you have already dropped of the white dummy or not
            }
            else {
              proximity_counter2 = 0;
            }
          }
          else{
            follow_line();
            proximity_counter2 ++;
          }
          if (proximity_counter2 > 3) {
            robot_sub_state = 9;
          }
        }
        else if (robot_sub_state == 9) {                         // reverses a little, helps with sweep angle to find next dummy
          if (reverse_again_timer == 0) {
            reverse_again_timer = tick_counter*tick_length;
          }
          else if (tick_counter*tick_length < reverse_again_timer + 2000) {
            drive_motor(left_motor,255,true);
            drive_motor(right_motor,255,true);
          }
          else {
            drive_motor(left_motor,0,false);
            drive_motor(right_motor,0,false);
            robot_sub_state = 0;
            robot_state = 2;
            what_dummy_am_I = 0;
            dummy_identity_decided = false;
          }
        }
      }
      else if (robot_state == 2){                                // finds the dummy. Starts on the line (advanced area)
        if (number_dummies_saved == 3){
          robot_state = 4;
        }
        else if (robot_sub_state == 0) {
          if (avoid_first_sweep_time == 0) {
            avoid_first_sweep_time = tick_counter*tick_length;
          }                                                      // first turn to avoid the dummy in the white box
          else if (tick_counter*tick_length < avoid_first_sweep_time + 500) {
            drive_motor(left_motor,255,true);
            drive_motor(right_motor,255,false);
          }
          else if (tick_counter*tick_length < avoid_first_sweep_time + 1500) {
            drive_motor(left_motor,0,false);
            drive_motor(right_motor,0,false);
          }
          else {
            robot_sub_state = 1;
          }
        }
        else if (robot_sub_state == 1){                          // runs point towards nearest dummy to find which side dummy is on
          if (point_towards_nearest_dummy(8200,0.8,255)) {
            robot_sub_state = 2;
            if ((int(percentage_of_sweep_completed * 10)%10)/10.0 <= 0.5) {
              dummy_side_right = false;
            }
            else {
              dummy_side_right = true;
            }
          }
        }
        else if (robot_sub_state == 2) {                         // approaches dummy
          if (home_dummy()) {
            robot_sub_state = 3;
          }
        }
        else if (robot_sub_state == 3) {
          if (delay_5s_start_time == 0){
            using_servos = false;
            delay_5s_start_time = tick_counter*tick_length;
          }
          else if (((tick_counter * tick_counter) > (-1+delay_5s_start_time)) and ((tick_counter * tick_length) < (delay_5s_start_time + five_seconds))){
            using_servos = false;
            what_dummy_am_I = identify_dummy();                  // identifies dummy using same technique as previously.
            if (what_dummy_am_I == 1) {
              readings_mod++;
            }
            else if (what_dummy_am_I == 2) {
              readings_mix++;
            }
            else if (what_dummy_am_I == 3) {
              readings_unm++;
            }
          }
          else if ((tick_length * tick_counter) > (delay_5s_start_time + five_seconds - 1) and not dummy_identity_decided) {
            if (readings_mod > readings_mix and readings_mod > readings_unm) {
              what_dummy_am_I = 1;
              dummy_identity_decided = true;                     // writes to LED pins depending on which dummy is found
              digitalWrite(LED2_PIN,HIGH);
              digitalWrite(LED3_PIN,HIGH);
            }
            else if (readings_mix > readings_mod and readings_mix > readings_unm) {
              what_dummy_am_I = 2;
              dummy_identity_decided = true;
              digitalWrite(LED2_PIN,HIGH);
              digitalWrite(LED3_PIN,LOW);
            }
            else if (readings_unm > readings_mix and readings_unm > readings_mod) {
              what_dummy_am_I = 3;
              dummy_identity_decided = true;
              digitalWrite(LED2_PIN,LOW);
              digitalWrite(LED3_PIN,HIGH);
            }
            readings_mod = 0;
            readings_mix = 0;
            readings_unm = 0;
          }
          else if ((tick_length * tick_counter) > (delay_5s_start_time + five_seconds - 1)){
            using_servos = true;
            picked_up_yet = pick_up_dummy();
          }
          if (picked_up_yet){                                    // identifies the dummy waits 5s and picks it up
            delay_5s_start_time = 0;
            robot_state = 3;
            robot_sub_state = 0;
            picked_up_yet = false;
            dummy_located = false;
            dummy_identity_decided = false;
            turn_timer = 0;
          }
        }
      }

      else if (robot_state == 3){                                //found second dummy, need to return it to its home, goes back to prime posiiton with dummy in arms
        if (robot_sub_state == 0) {
          if (turn(angle6,false)) {
            robot_sub_state = 1;
          }
        }
        else if (robot_sub_state == 1) {
          if (what_dummy_am_I == 1) {
            if (turn_onto_line(dummy_side_right)) {              // Use turn onto line algorithm: Drive until on line, continue for 1s, turn the correct direction (according to dummy_side_right) until back onto line
              robot_sub_state = 0;
              robot_state = 1;
            }
          }
          else {
            if (turn_onto_line(not dummy_side_right)) {
              robot_sub_state = 1;
              robot_state = 1;
            }
          }
        }
      }
      else if (robot_state == 4){                                // this is the go home part of the algorithm
        if (robot_sub_state == 0){                               // turns around so facing home
          if (turn(angle7)){
            robot_sub_state = 1;
          }
        }
        if (robot_sub_state == 1){
          follow_line_output = follow_line();                    // the 2.5s timer is to get the robot into the box after it crosses the line
          if (drive_1s_2 != 0 and tick_length * tick_counter < drive_1s_2 + 2500) {
            drive_motor(left_motor,255,false);
            drive_motor(right_motor,255,false);
          }
          else if (drive_1s_2 != 0 and tick_length * tick_counter > drive_1s_2 + 2499) {
            drive_motor(left_motor,0,false);
            drive_motor(right_motor,0,false);
            robot_state = 5;
            robot_sub_state = 0;
          }
          else if (follow_line_output and hit_cross_roads == 2){
            if (drive_1s_2 == 0){
              drive_1s_2 = tick_counter * tick_length; 
            } 
          }
          else if (follow_line_output and hit_cross_roads == 0) {
            hit_cross_roads = 1;                                 // hit first cross roads but want to continue
          }
          else if (hit_cross_roads == 1 and not follow_line_output){
            hit_cross_roads = 2;                                 // hit second line now want to continue only a little bit
          }
        }
      }
    }
    tick_counter ++;                                             // Increment tick counter
    delay(tick_length - (millis() % tick_length));               // Delay the remaining milliseconds of the tick to keep tick rate constant (as long as computer fast enough)
  }
}
