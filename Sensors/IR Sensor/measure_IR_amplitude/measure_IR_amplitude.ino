
int measure_IR_amplitude(int IR_PIN, int samples) {
  // Function to record voltage amplitude of IR sensor 10% duty cycle signal
  // IR_PIN must be an analog pin for any non-binary results.
  // Wave is ~6ms long, and samples take ~ 112us, so set samples to AT LEAST >54 to ensure peak is included in measurement (preferably > 108)
  // CODE IS BLOCKING TO ALLOW FAST SAMPLING RATE - blocking time will increase linearly with samples
  
  int max_val;                            // Variable to store max. voltage measured
  int new_val;                            // Variable to store current measurement
  for (int i=0; i<samples; i++) {
    new_val = analogRead(IR_PIN);         // Measure current voltage from photodiode
    if (new_val > max_val) {
      max_val = new_val;                  // Update maximum recorded value if it is new highest
    }
  }
  return max_val;
}
