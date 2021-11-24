void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(11, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(digitalRead(11));
}
