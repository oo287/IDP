// REQUIRED LIBRARY IMPORT
#include <Servo.h>

// REQUIRED OBJECT INITIALISATIONS
Servo servo_1;
int SERVO_1_PIN = 9;                // Two servo pins are 9 and 10 for adafruit motorshield v2


void setup() {
  // REQUIRED SETUP CODE
  servo_1.attach(SERVO_1_PIN);      // Indicate servo is attached to this pin
}


void set_servo_angle(Servo servo, int angle) {      // Function that writes angle to a generic servo object, clipped at specified mechanical limits
  int lower_limit = 20;                             // Degrees, writing a lower angle will produce no movement regardless of start position
  int upper_limit = 160;                            // Degrees, writing a higher angle will produce no movement regardless of start position
  
  if (angle < lower_limit) {                        // Clip input to the specified limits
    angle = lower_limit;
  }
  else if (angle > upper_limit) {
    angle = upper_limit;
  }

  servo.write(angle);                               // Make servo move to that angle (will enforce this position without needing to recall function)
}



// SERVO WILL TAKE TIME TO MOVE - ANY PROGRAMS USING THIS FUNCTION MUST WAIT THE APPROPRIATE TIME BEFORE ASSUMING SERVO HAS REACHED FINAL POSITION.
// A servo will actively hold its position once set.
