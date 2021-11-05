#include <Wire.h>                                  // Use Wire to communicate using I2C with the MPUa
#include <MemoryFree.h>                            // Library to detect when EEPROM memory runs out for recording flight data
#include <BMx280I2C.h>                             // Library to communicate with BMP280 Pressure sensor

byte rocket_state = 0;                             // Stores state the rocket is in currently. Idle (0), Awaiting Launch (1), In Flight (2), Deploy Parachute (3)

const int tick_length = 20;                        // Length (in ms) of each tick
long tick_counter = 0;                             // Stores how many ticks have occured since powerup of arduino (millis() % tick_length)

bool LED_state = false;                            // Stores the state of the LED, false = OFF, true = ON

const int controlPin = A0;                         // Pin used for switching from state 0 (IDLE) to state 1 (Awaiting Launch)
const int motorPin = 1;                            // Pin used for activating the motor via a MOSFET
int controlPinVoltage = 0;                         // Voltage variable for control pin
byte bootUpCounter = 0;                            // Counter for ensuring voltage input not due to static
const int bootUpCutoff = 716;                      // Minimum voltage required to initiate bootup (716-1023 (3.5-5 Volts)) (0-5V -> 0-1023)
const int windUpCutoff = 61;                       // Minimum voltage to windUp (61-715 (0.3-3.5 Volts))

int deployCounter = 0;                             // Counter to keep track of how long the motor has been running when deploying the chute
const int deployPulseLength = 60;                  // Number of ticks to wind the motor when parachute deploy is triggered
const int numberOfDeployPulses = 5;                // Number of times the motor will be turned on to deploy the chute
int pulsesDone = 0;                                // Keep track of how many parachute deploy pulses have taken place

long rawAccelX, rawAccelY, rawAccelZ;              // For direct digital MPU outputs
float accelX, accelY, accelZ, accelMag2;           // For converted values into m/s^2 (accelMag2 is magnitude squared of acc vector)
float pressure = 101325.0;                         // For pressure values from BMP280
float altitude = 0.0;                              // ASL Altitude, normalised at launch to 0m
float altitudeOffset = 0.0;                        // Offset applied to ASL altitude to measure AGL altitude
bool altitudeCalibrated = false;                   // Records if the ASL offset has been calculated yet
float altitudeCutoff = 5;                          // Minimum altitude AGL to confirm liftoff
byte liftoffCounter = 0;                           // Counts to ensure liftoff isn't detected from a glitch in acceleration readings
byte liftoffCutoff = 5;                            // Minimum number of ticks required to trigger liftoff from acceleration or altitude rise above altitudeCutoff

float peakAltitude = 0.0;                          // Records peak altitude to determine when apogee is reached
byte apogeeCounter = 0;                            // Counts the ticks after maximum altitude is reached, to ensure not just noise
const byte apogeeCutoff = 50;                      // Minimum number of ticks required to confirm apogee reached

float tempPeakAltitude = 0.0;                      // Used to display the peak altitude in reverse binary with the Arduino LED
bool binaryOutput = false;                         // Used to keep track of current output value

int liftoffTime = 0;                               // Number of ticks elapsed at liftoff (used for hard backup of apogee reached)
int backupChuteDelay = 200;                        // Number of ticks after launch detected to command parachute deploy (used as backup to apogee detection)

long rawRotX, rawRotY, rawRotZ;                    // Digital inputs
float rotX, rotY, rotZ;                            // Values in rad/s
float angleX, angleY, angleZ;                      // Values in radians, about X,Y,Z axes
float rotCalX=0.0, rotCalY=0.0, rotCalZ=0.0;       // Calibration values to offset steady-state non-zero values for angular velocity (rad/s)
bool gyrosCalibrated = false;                      // Records whether the gyros have been calibrated yet or not

const int gyroSensitivity = 0b00011000;            // 0b000XX000, XX = 00,01,10,11 => Range = +/- 4.36, 8.73, 17.45, 34.91 radians/second (250,500,1000,2000 deg/sec)
const int accSensitivity = 0b00011000;             // 0b000XX000, XX = 00,01,10,11 => Range = +/- 19.61, 39.23, 78.45, 156.91 m/s^2 (2,4,8,16 g)
float gyroLSB, accLSB;                             // Used to convert digital values to true measurements

const int batteryVoltageMonitorPin = A7;               // Connected to V_IN to monitor battery voltage. If too low, do not allow flight
float batteryVoltage;                              // Value of battery voltage (between)

BMx280I2C bmx280(0x76);

void setup() {

  Serial.begin(9600);

  rocket_state = 0;
  LED_state = false;
  gyrosCalibrated = false;
  altitudeCalibrated = false;

  pinMode(LED_BUILTIN, OUTPUT);                    // Initialise LED Pin
  pinMode(controlPin,INPUT);                       // Set Control Pin to INPUT
  pinMode(batteryVoltageMonitorPin,INPUT);         // Set BVM Pin to INPUT
  pinMode(motorPin,OUTPUT);                        // Initialise Motor output pin
                     
  Wire.begin();                                    // Initialise wire package

  Serial.println("Startup Attempted");

  if (!bmx280.begin())
  {
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    while (1);
  }

  bmx280.resetToDefaults();
  
  if (gyroSensitivity == 0b00000000) gyroLSB = 131.0;
  else if (gyroSensitivity == 0b00001000) gyroLSB = 65.5;
  else if (gyroSensitivity == 0b00010000) gyroLSB = 32.75;
  else if (gyroSensitivity == 0b00011000) gyroLSB = 16.375;
  
  if (accSensitivity == 0b00000000) accLSB = 16384.0;
  else if (accSensitivity == 0b00001000) accLSB = 8192.0;
  else if (accSensitivity == 0b00010000) accLSB = 4096.0;
  else if (accSensitivity == 0b00011000) accLSB = 2048.0;
  
  setupMPU();
  setupBMP();
}

void setupMPU() {
  Wire.beginTransmission(0b1101001);               // Address (I2C) of the MPU, LSB must equal AD0 pin value
  Wire.write(0x6B);                                // Register to configure MPU mode (Sleep, Cycle etc.)
  Wire.write(0b00000000);                          // Sets SLEEP bit to 0 to wake MPU up (boots up in SLEEP mode)
  Wire.endTransmission();

  Wire.beginTransmission(0b1101001);
  Wire.write(0x1B);                                // GYRO_CONFIG Register (Self tests, Measurement range setting)
  Wire.write(gyroSensitivity);                     // 0b000XX000, XX = 00,01,10,11 => Range = +/- 4.36, 8.73, 17.45, 34.91 radians/second (250,500,1000,2000 deg/sec)
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101001);
  Wire.write(0x1C);                                // ACCEL_CONFIG Register (Self tests, Measurement range setting)
  Wire.write(accSensitivity);                      // 0b000XX000, XX = 00,01,10,11 => Range = +/- 19.61, 39.23, 78.45, 156.91 m/s^2 (2,4,8,16 g)
  Wire.endTransmission();
}

void setupBMP() {
  Wire.beginTransmission(0b1110110);               // Address (I2C) of the BMP
  Wire.write(0xF4);
  Wire.write(0b00010011);                          // Temperature oversampling setting (XXX: 000-101 0x, 1x, 2x, 4x, 8x, 16x), Pressure oversampling setting (XXX: 000-101 0x, 1x, 2x, 4x, 8x, 16x), Sensor Mode (XX: 00 Sleep, 01/10 Forced, 11 Normal)
  Wire.endTransmission();                          // Temp resolution: 0.005 C, halved with each sampling rate doubling. Press resolution: 2.62 Pa, halved with each sampling rate doubling.

  Wire.beginTransmission(0b1110110);               // Address (I2C) of the BMP
  Wire.write(0xF5);
  Wire.write(0b00101001);                          // Sampling delay (XXX: 000 0.5ms, 001 62.5ms, 010 125ms, doubling up to 4000ms), IIR Filter Setting (XXX: 000-100 -> 1,2,4,8,16), Reserved (0), SPI 3/4 Wire Setting (X)
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101001);
  Wire.write(0x3B);                                // First of 6 Registers that store acceleration values
  Wire.endTransmission();
  Wire.requestFrom(0b1101001,6);                   // Take values from registers 3B,3C,3D,3E,3F,40
  while(Wire.available() < 6) {                    // Wait for all 6 data to come in
    digitalWrite(LED_BUILTIN, HIGH);               // Orange LED on means NO DATA
  }
  digitalWrite(LED_BUILTIN, LOW);
  rawAccelX = Wire.read()<<8|Wire.read();          // Store first two bytes into rawAccelX
  rawAccelY = Wire.read()<<8|Wire.read();          // Store next two bytes into rawAccelY
  rawAccelZ = Wire.read()<<8|Wire.read();          // Store next two bytes into rawAccelZ
  processAccelData();                              // Converts raw data into accelerations in m/s^2
}

void processAccelData() {
  accelX = rawAccelX / accLSB;                     // Convert to g Force
  accelX = accelX * 9.80665;                       // Convert to m/s^2
  accelY = rawAccelY / accLSB;
  accelY = accelY * 9.80665;
  accelZ = rawAccelZ / accLSB;
  accelZ = accelZ * 9.80665;
  accelMag2 = accelX*accelX + accelY*accelY + accelZ*accelZ;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101001);
  Wire.write(0x43);                                // First of 6 Registers that store rotation values
  Wire.endTransmission();
  Wire.requestFrom(0b1101001,6);                   // Take values from registers 43,44,45,46,47,48
  while(Wire.available() < 6);                     // Wait for all 6 data to come in
  rawRotX = Wire.read()<<8|Wire.read();            // Store first two bytes into rawRotX
  rawRotY = Wire.read()<<8|Wire.read();            // Store next two bytes into rawRotY
  rawRotZ = Wire.read()<<8|Wire.read();            // Store next two bytes into rawRotZ
  processGyroData();                               // Converts raw data into angular rate in rad/s
}

void processGyroData() {
  rotX = rawRotX / gyroLSB;                        // Convert to deg/sec
  rotX = rotX / 57.29577951;                       // Convert to rad/sec
  rotY = rawRotY / gyroLSB;
  rotY = rotY / 57.29577951;
  rotZ = rawRotZ / gyroLSB;
  rotZ = rotZ / 57.29577951;

  if (gyrosCalibrated) {
    rotX = rotX - rotCalX;                         // Subtract the calibration factor to normalise the value
    rotY = rotY - rotCalY;
    rotZ = rotZ - rotCalZ;
  }
}

void recordPressure() {
  bmx280.measure();
  while (!bmx280.hasValue()) {
    delay(0.1);
  }
  pressure = bmx280.getPressure();
}

void calculateAltitude() {
  altitude = 44330.76*(1-pow((pressure/101325.0),(0.1902630958)));
  if (altitudeCalibrated) {
    altitude = altitude - altitudeOffset;
  }
}

void calibrateGyros() {
  for (int i=0; i<10; i++) {
    recordGyroRegisters();
    rotCalX = rotCalX + rotX;
    rotCalY = rotCalY + rotY;
    rotCalZ = rotCalZ + rotZ;
    delay(tick_length);
  }
  
  rotCalX = rotCalX/10;
  rotCalY = rotCalY/10;
  rotCalZ = rotCalZ/10;

  gyrosCalibrated = true;
  
}

void calibrateAltitude() {

  if (not altitudeCalibrated) {
    altitudeOffset = 0.0;

    for (int i=0; i<10; i++) {
      recordPressure();
      calculateAltitude();
      altitudeOffset = altitudeOffset + altitude;
      delay(tick_length);
    }
  
    altitudeOffset = altitudeOffset/10;
    altitudeCalibrated = true;
    //Serial.println("Altitude Calibrated!");
  } 
}

void integrateRotation() {
  angleX = angleX + rotX*tick_length*0.001;
  angleY = angleY + rotY*tick_length*0.001;
  angleZ = angleZ + rotZ*tick_length*0.001;
}

void printData() {
  Serial.print("Gyro (rad/s)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" angX=");
  Serial.print(angleX);
  Serial.print(" angY=");
  Serial.print(angleY);
  Serial.print(" angZ=");
  Serial.print(angleZ);
  Serial.print(" Accel (m/s^2)");
  Serial.print(" X=");
  Serial.print(accelX);
  Serial.print(" Y=");
  Serial.print(accelY);
  Serial.print(" Z=");
  Serial.print(accelZ);
  Serial.print(" Mag2=");
  Serial.print(accelMag2);
  Serial.print(" P=");
  Serial.print(pressure);
  Serial.print(" h=");
  Serial.println(altitude);
}

void loop() {

  controlPinVoltage = analogRead(controlPin);      // Read voltage for Control Pin (0-5V -> 0-1023)

  if (controlPinVoltage < bootUpCutoff) {
    if (controlPinVoltage >= windUpCutoff) {
      digitalWrite(motorPin,HIGH);                 // Wind the string in
    }
    else if (rocket_state == 3 and deployCounter < deployPulseLength and pulsesDone < numberOfDeployPulses) {
      digitalWrite(motorPin,HIGH);                 // Deploy
      deployCounter++;
    }
    else if (rocket_state == 3 and deployCounter < 1.2*deployPulseLength and pulsesDone < numberOfDeployPulses) {               
      digitalWrite(motorPin,LOW);                  // Stop motor
      deployCounter++;
    }
    else if (rocket_state == 3 and pulsesDone < numberOfDeployPulses) {
      deployCounter = 0;
      digitalWrite(motorPin,HIGH);
      pulsesDone++;
      
    }
    else {
      digitalWrite(motorPin,LOW);
    }
  }


  if (rocket_state == 1 or rocket_state == 2) {    // Only collect input data if waiting to detect launch or in flight (no data necessary after parachute deploy)

    recordAccelRegisters();
    recordPressure();
    calculateAltitude();
    recordGyroRegisters();
    
    //integrateRotation();
    //printData(); 

//    temp_tickspeed_counter = temp_tickspeed_counter + 1;                  // Just taking readings: 5.755ms per cycle (174 Hz)
//    if (temp_tickspeed_counter == 1) {
//      Serial.println("1");
//    }
//    else if (temp_tickspeed_counter >= 2001) {
//      Serial.println("2001");
//      temp_tickspeed_counter = 0;
//    }
//    Serial.println(temp_tickspeed_counter);
  }

  if (rocket_state == 0) {                         // If idle, run IDLE_BLINK program (heartbeat LED flash)

    if (tick_counter % 65 >= 25) {                 // IDLE_BLINK program below
      LED_state = false;
      digitalWrite(LED_BUILTIN,LOW);
    }
    else if (tick_counter % 65 >= 20) {
      LED_state = true;
      digitalWrite(LED_BUILTIN,HIGH);
    }
    else if (tick_counter % 65 >= 5) {
      LED_state = false;
      digitalWrite(LED_BUILTIN,LOW);
    }
    else {
      LED_state = true;
      digitalWrite(LED_BUILTIN,HIGH);
    }
    
    //Serial.println(bootUpPinVoltage);
    
    if (controlPinVoltage >= bootUpCutoff) {       // Bootup requires the signal to be held for 1s, so a counter is used
      bootUpCounter++;
    }
    
    if (bootUpCounter >= 50) {                     // Count for multiple ticks (1 second) to ensure signal not a glitch
      LED_state = true;
      digitalWrite(LED_BUILTIN,HIGH);
      rocket_state = 1;
      calibrateGyros();
      calibrateAltitude();
    }
  }

  else if (rocket_state == 1) {
    
    if (tick_counter % 20 >= 10) {                 // ARMED_BLINK program below
      LED_state = false;
      digitalWrite(LED_BUILTIN,LOW);
    }
    else {
      LED_state = true;
      digitalWrite(LED_BUILTIN,HIGH);
    }

//    if (accelMag2 > 1000 or (altitude > altitudeCutoff and altitudeCalibrated == true)) {
    if (altitude > altitudeCutoff and altitudeCalibrated == true) {
      liftoffCounter++;
      //Serial.println("Detection spike");
    }
    else {
      liftoffCounter = 0;
    }

    if (liftoffCounter > liftoffCutoff) {
      rocket_state = 2;
      liftoffCounter = 0;
      liftoffTime = tick_counter;
    }
  }

  else if (rocket_state == 2) {
    digitalWrite(LED_BUILTIN,HIGH);              // INFLIGHT_BLINK program (solid ON)

    if (altitude > peakAltitude) {
      peakAltitude = altitude;
    }
    if (altitude < peakAltitude) {               // Suggests beyond apogee
      apogeeCounter++;
    }
    if (apogeeCounter > apogeeCutoff) {          // Confirm beyond apogee and command parachute deploy
      rocket_state = 3;
      //peakAltitude = random(4,200);
    }
    if (tick_counter - liftoffTime > backupChuteDelay) {
      rocket_state = 3;
      //peakAltitude = random(4,200);
    }
  }

  else if (rocket_state == 3) {

    if (tick_counter % 450 == 0) {
      tempPeakAltitude = floor(peakAltitude);
      Serial.println(peakAltitude);
    }
    else if (tick_counter % 450 < 50) {                 // Program to display peak altitude in binary (in reverse)
      LED_state = true;
      digitalWrite(LED_BUILTIN,HIGH);
    }
    else if (tick_counter % 50 == 0) {
      tempPeakAltitude = tempPeakAltitude / 2;
      if (tempPeakAltitude == floor(tempPeakAltitude)) {
        binaryOutput = false;
      }
      else {
        binaryOutput = true;
      }
      tempPeakAltitude = floor(tempPeakAltitude);
      LED_state = false;
      digitalWrite(LED_BUILTIN,LOW);
    }
    else if (binaryOutput == false) {
      if (tick_counter % 50 < 20) {
        LED_state = false;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if (tick_counter % 50 < 30) {
        LED_state = true;
        digitalWrite(LED_BUILTIN,HIGH);
      }
      else {
        LED_state = false;
        digitalWrite(LED_BUILTIN,LOW);
      }
    }
    else {
      if (tick_counter % 50 < 10) {
        LED_state = false;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if (tick_counter % 50 < 20) {
        LED_state = true;
        digitalWrite(LED_BUILTIN,HIGH);
      }
      else if (tick_counter % 50 < 30) {
        LED_state = false;
        digitalWrite(LED_BUILTIN,LOW);
      }
      else if (tick_counter % 50 < 40) {
        LED_state = true;
        digitalWrite(LED_BUILTIN,HIGH);
      }
      else {
        LED_state = false;
        digitalWrite(LED_BUILTIN,LOW);
      }
    } 
  }
  
  tick_counter++;
  delay(tick_length - (millis() % tick_length));

}
