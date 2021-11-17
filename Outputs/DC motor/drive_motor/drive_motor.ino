// REQUIRED LIBRARY IMPORT
#include <Adafruit_MotorShield.h>                                // Import motor shield library

// REQUIRED OBJECT INITIALISATIONS
Adafruit_MotorShield AFMS = Adafruit_MotorShield();              // Create motorshield object called 'AFMS' with default I2C address. Put I2C address in brackets if different address needed
Adafruit_DCMotor *left_motor = AFMS.getMotor(LM_port);           // Initialise left  motor using assigned motor shield port
Adafruit_DCMotor *right_motor = AFMS.getMotor(RM_port);          // Initialise right motor using assigned motor shield port


void setup() {
  // REQUIRED SETUP CODE
  while (!AFMS.begin()) {                                        // Wait until motorshield successfully initialised
    Serial.println("Failed to find motorshield.");
    delay(1000);                 
  }
  Serial.print("Motorshield successfully initialised.");         // Infamous message that will haunt your nightmares (displays repeatedly if Arduino crashing (check drive_motor() uses!))

  left_motor->setSpeed(0);                                       // Initialise left motor
  left_motor->run(FORWARD);
  left_motor->run(RELEASE); 

  right_motor->setSpeed(0);                                      // Initialise right motor
  right_motor->run(FORWARD);
  right_motor->run(RELEASE); 

}



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
