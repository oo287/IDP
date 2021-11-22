const int buttonpin = 4;                          // The digital pin that will receive a signal of 0/5 V. Change as appropriate.
int awake = 0;                                    // Global variable to store whether the robot is awake, initialized as LOW (zero).

void setup() {
  pinMode(INPUT, buttonpin);                       // Initialise buttonpin to be used as digital input.
  Serial.begin(9600);
}

void loop() {
  int button = 0;                                  // Variable to store the state of the button, initialized as LOW (zero).
  button = digitalRead(buttonpin);                 // Read the digital signal from buttonpin, store it in variable button.
  if (button == 1) { awake = 1 ;}                  // As soon as the button is pressed, the variable awake is set to HIGH (1).
}

// The main code should only run if the global variable awake is HIGH (1). 
