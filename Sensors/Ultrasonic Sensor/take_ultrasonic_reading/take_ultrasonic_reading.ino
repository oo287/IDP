
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
