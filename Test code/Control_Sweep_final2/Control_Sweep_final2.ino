/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

float timer = 0;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int angle;


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    angle = Serial.parseInt();
  }
  myservo.write(angle);              // tell servo to go to position in variable 'pos'
  delay(15);                       // waits 15ms for the servo to reach the position

  timer += 0.01
  

}
