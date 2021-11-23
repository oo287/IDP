

int MOD_PIN = 6;
int samples = 4000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MOD_PIN, INPUT);
}

float pwm_summer(int MOD_PIN, int samples) {
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum = sum + (int) digitalRead(MOD_PIN);
    delayMicroseconds(10);
  }
  return (float(sum)/float(samples));
}



void loop() {
  // put your main code here, to run repeatedly:
  float total = pwm_summer(MOD_PIN, samples);
  Serial.println(total);
  if (total > 0.7 and total < 0.85) {
    Serial.println("Modulated");
  } else if (total > 0.85 and total < 0.95) {
    Serial.println("Mixmodulated");
  } else if (total > 0.95) {
    Serial.println("Unmodulated");
  } else {
    Serial.println("???");
  }
}
