int sensorPin = A0;
int sensorPin2 = A1;
int sensorPin3 = A2; 
int sensorPin4 = A3; 
// select the input pin for the potentiometer

int sensorValue = 0;  // variable to store the value coming from the sensor
int sensorValue2 = 0;  // variable to store the value coming from the sensor
int sensorValue3 = 0;  // variable to store the value coming from the sensor
int sensorValue4 = 0;  // variable to store the value coming from the sensor

void setup() {
  Serial.begin(9600);           //Start serial and set the correct Baud Rate
}

void loop() {
  sensorValue = analogRead(sensorPin);
  Serial.print("Sensor 1: ");
  Serial.print(sensorValue);
  Serial.print(" ");
  sensorValue2 = analogRead(sensorPin2);
  Serial.print("Sensor 2: ");
  Serial.print(sensorValue2);
  Serial.print(" ");
  sensorValue3 = analogRead(sensorPin3);
  Serial.print("Sensor 3: ");
  Serial.println(sensorValue3);
  Serial.print(" ");
  sensorValue4 = analogRead(sensorPin3);
  Serial.print("Sensor 4: ");
  Serial.println(sensorValue4);
}
