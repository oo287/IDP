/*
 * IRremote: IRsendRawDemo - demonstrates sending IR codes with sendRaw
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 *
 * IRsendRawDemo - added by AnalysIR (via www.AnalysIR.com), 24 August 2015
 *
 * This example shows how to send a RAW signal using the IRremote library.
 * The example signal is actually a 32 bit NEC signal.
 * Remote Control button: LGTV Power On/Off. 
 * Hex Value: 0x20DF10EF, 32 bits
 * 
 * It is more efficient to use the sendNEC function to send NEC signals. 
 * Use of sendRaw here, serves only as an example of using the function.
 * 
 */



#include <EEPROM.h>

#define TIMER_PWM_PIN 6 //defined in the library.


unsigned long previousMillis = 0;        // will store last time LED was updated

int interval ; 
int mode;          
const int ledPin =  LED_BUILTIN;// the number of the LED pin
int serial_in =1;
int ledState = LOW;  

#if defined(__AVR_ATmega4809__)
  #define IR_USE_TIMER_4809_1     //  tx = pin 24
  
#else
  #define IR_USE_TIMER2     // tx = pin 3 nano. 
  #define TIMER_ENABLE_PWM    (TCCR2A |= _BV(COM2B1))
  #define TIMER_DISABLE_PWM   (TCCR2A &= ~(_BV(COM2B1)))
  #define USECPERTICK    50
  #define TIMER_COUNT_TOP  (SYSCLOCK * USECPERTICK / 1000000)
  
  #define TIMER_PWM_PIN  3  
  #define TIMER_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
  TCCR2A               = _BV(WGM20); \
  TCCR2B               = _BV(WGM22) | _BV(CS20); \
  OCR2A                = pwmval; \
  OCR2B                = pwmval / 3; \
})
#endif

#if defined(IR_USE_TIMER_4809_1)
// ATmega4809 TCB0
#define TIMER_RESET          TCB0.INTFLAGS = TCB_CAPT_bm
#define TIMER_ENABLE_PWM     (TCB0.CTRLB |= TCB_CCMPEN_bm)
#define TIMER_DISABLE_PWM    (TCB0.CTRLB &= ~(TCB_CCMPEN_bm))
#define TIMER_ENABLE_INTR    (TCB0.INTCTRL = TCB_CAPT_bm)
#define TIMER_DISABLE_INTR   (TCB0.INTCTRL &= ~(TCB_CAPT_bm))
#define TIMER_INTR_NAME      TCB0_INT_vect
#define TIMER_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
  TCB0.CTRLB = TCB_CNTMODE_PWM8_gc; \
  TCB0.CCMPL = pwmval; \
  TCB0.CCMPH = pwmval / 2; \
  TCB0.CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm); \
})
#define TIMER_COUNT_TOP      ((SYSCLOCK * USECPERTICK / 1000000))
#define TIMER_CONFIG_NORMAL() ({ \
  TCB0.CTRLB = (TCB_CNTMODE_INT_gc); \
  TCB0.CCMP = TIMER_COUNT_TOP; \
  TCB0.INTCTRL = TCB_CAPT_bm; \
  TCB0.CTRLA = (TCB_CLKSEL_CLKDIV1_gc) | (TCB_ENABLE_bm); \
})
#define TIMER_PWM_PIN        6  /* Nano Every, Uno WiFi Rev2 */ 
#endif



#define SYSCLOCK  F_CPU

  int toggle =0;

void setup()
{
 pinMode(TIMER_PWM_PIN, OUTPUT);
 pinMode(ledPin, OUTPUT);
 digitalWrite(TIMER_PWM_PIN, LOW);
  int khz = 38; // 38kHz carrier frequency for the NEC protocol
  

 TIMER_CONFIG_KHZ(khz);//CALLS TIMER_CONFIG_KHZ
 
  

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  checkMode();
 
 
  
}

void loop() {

     

  if(mode==1)
  {
    TIMER_ENABLE_PWM;
    delayMicroseconds(600);
    TIMER_DISABLE_PWM;
    interval = 1000;
  }
  
    
  else if(mode==2)
  {
    digitalWrite(TIMER_PWM_PIN, HIGH);
    delayMicroseconds(600);
    digitalWrite(TIMER_PWM_PIN, LOW);
    interval = 2000;
  }
    
   else
   {
    if (toggle == 0)
    {
      digitalWrite(TIMER_PWM_PIN, HIGH);
      delayMicroseconds(600);
      digitalWrite(TIMER_PWM_PIN, LOW);
      toggle = 1;
    }
    else {
      TIMER_ENABLE_PWM;
      delayMicroseconds(600);
      TIMER_DISABLE_PWM;
      toggle = 0;
    }
    interval = 100;
   }
    
   
   delayMicroseconds(5400);

   unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
        // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  
} 
}

void checkMode() {
   Serial.println("Mode status after restart: ");
   mode = EEPROM.read(0);
   mode = mode+1;   
   if(mode > 3)
   {
    mode =1;
   }
   EEPROM.update(0, mode);
   if(mode == 1) {
    Serial.println ("Mode 1");
    
   } 
   else if(mode == 2) {
    Serial.println ("Mode 2");
    
   }
   else{
    mode =3;
    Serial.println("Mode 3");
   }
   
}
