
// digitalRead takes about 3.42 seconds
// At least two waves must be sampled, as every two adjacent mixmodulated waves are combined into one
// => 12ms sampling time at least
// => around 3509 samples
// More samples will skew all averages towards 1.00 making it harder to distinguish, but will ensure more consistent averaging.

// analogRead can also be used to increase spread of averages, but sample rate is much slower (112us) and requires analog pin.

// REQUIRED VARIABLES
int MOD_PIN = 6;

void setup() {
  // REQUIRED SETUP
  Serial.begin(9600);
  pinMode(MOD_PIN, INPUT);
}


int pwm_summer(int MOD_PIN) {
  // Function to return modulation type as enumerated integer (1=mod, 2=mix, 3=unmod, 4=bad)
  
  // Number of times to sample the IR reading (1 sample ~=~ 3.42us)
  int samples = 3509;                               // Changing this will change the boundary conditions! (averages get skewed!)
  float lower_mod = 0.85;                            // Lower boundary of average value for modulated signal
  float lower_mix = 0.91;                           // Lower boundary of average value for mixmodulated signal (and upper for modulated)
  float lower_unm = 0.96;                           // Lower boundary of average value for unmodulated signal (and upper for mixmodulated)
  
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum = sum + (int) digitalRead(MOD_PIN);         // Continually sample value of wave, and add up total
  }
  float total = (float(sum)/float(samples));        // Calculate mean value of wave
  
  //Serial.print(total);
  //Serial.print(" ");
  if (total >= lower_mod and total < lower_mix) {
    return 1;                                       // Modulated
  } else if (total >= lower_mix and total < lower_unm) {
    return 2;                                       // Mixmodulated
  } else if (total >= lower_unm) {
    return 3;                                       // Unmodulated
  } else {
    return 0;                                       // Something weird???
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pwm_summer(MOD_PIN));
}
