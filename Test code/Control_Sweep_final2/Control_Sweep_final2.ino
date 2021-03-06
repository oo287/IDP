/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo2;
// twelve servo objects can be created on most boards

int angle = 20;


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(8);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    angle = Serial.parseInt();
  }
  myservo.write(angle);              // tell servo to go to position in variable 'pos'
  myservo2.write(120);
  delay(15);                       // waits 15ms for the servo to reach the position

}
