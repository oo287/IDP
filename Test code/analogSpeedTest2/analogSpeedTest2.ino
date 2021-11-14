// Measurement test to see if amplitude of 10% duty cycle can be measured consistently.

int IR_PIN = A5;              // Analog pin that is measuring
unsigned long start_time;     // Time in microseconds that measurement starts
unsigned long stop_time;      // Time in microseconds that measurement ends


// Wave is ~6ms long, and samples take ~ 112us, so set samples to AT LEAST >54 to ensure peak is recorded (preferably > 108)
const int samples = 600;      // Number of measurements to take (duration of all samples is recorded)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);     // Set input pin to high impedance (IMPORTANT!!! Pushing voltage across pin set to low impedance can damage it)
}

void loop() {
  // put your main code here, to run repeatedly:
  start_time = micros();                  // Record start time
  // Initialise variables outside of loop to reduce overheads
  int max_val;                            // Variable to store max. voltage measured
  int new_val;                            // Variable to store current measurement
  for (int i=0; i<samples; i++) {
    new_val = analogRead(IR_PIN);         // Measure current voltage from photodiode
    if (new_val > max_val) {
      max_val = new_val;                  // Update maximum recorded value if it is new highest
    }
  }
  stop_time = micros();                   // Record end time
  Serial.print("Duration (us): ");
  Serial.print(stop_time - start_time);   // Output the total measurement duration
  Serial.print(" ");
  Serial.println(max_val);                // Output the recorded maximum voltage (should be voltage of "on" section, proportional to IR sensed)
}
