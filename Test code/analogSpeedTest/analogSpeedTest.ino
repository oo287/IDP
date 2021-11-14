// Measurement test to see if 10% duty cycle of wave can be resolved.

int IR_PIN = A5;              // Analog pin that is measuring
unsigned long start_time;     // Time in microseconds that measurement starts
unsigned long stop_time;      // Time in microseconds that measurement ends

const int samples = 600;      // Number of measurements to take (duration of all samples is recorded)
int waveform[samples];        // Array of stored measurements

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);     // Set input pin to high impedance (IMPORTANT!!! Pushing voltage across pin set to low impedance can damage it)
}

void loop() {
  // put your main code here, to run repeatedly:
  start_time = micros();                  // Record start time
  for (int i=0; i<samples; i++) {         // Loop n times
    waveform[i] = analogRead(IR_PIN);     // Record measurement
  }
  stop_time = micros();                   // Record end time
  Serial.print("Duration (us): ");
  Serial.print(stop_time - start_time);   // Print total duration to Serial
  Serial.print(" ,");

  for (int i=0; i<samples; i++) {         // Iterate through all measurements and print them
    Serial.print(waveform[i]);
    Serial.print(" ,");
  }
  Serial.println();
  Serial.println("Complete!");
}


// Results!!!! analogRead takes ~112us pretty consistently, getting about 6 samples per "on" portion of cycle.
// Make sure all arduinos are grounded directly, to avoid a strong smothering 50Hz noise from mains.
// Apparently output impedances of >10k should be avoided, as a capacitor must charge inside the analog pin to complete measurement.
