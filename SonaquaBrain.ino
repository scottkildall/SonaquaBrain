/*********************************************************
    Sonaqua Brain
 
    Written by Scott Kildall
    www.kildall.com
    March 2018

   This uses the MPR221 12-channel touch capacitance sensor.
   
   A simple Arudino sketch that uses the Adafruit LED backpack hooked to a MPR221 Touch Capacitance board by AdaFruit.

  This uses all 12 inputs and displays the results, as a binary sequence on the LED backpack, e.g.

  if no sensors are on, the value = 0
  if pin 0 is pressed down, value = 1
  if pin 1 is pressed down, value = 2
  if pin 0 and 1 is pressed down, value = 3

  if the capacitance sensor isn't working, it will display an error of 9999

  Adafruit GitHub repos for their libraries are here:
  https://github.com/adafruit/Adafruit_LED_Backpack
  https://github.com/adafruit/Adafruit_MPR121

  These need to be copied into the Arduino/libraries folder.

**********************************************************/

#include <Wire.h>
#include "Adafruit_MPR121.h"
#include "Adafruit_LEDBackpack.h"

#include "MSTimer.h"

//-- if autoplay mode = true, then no interactive
boolean bAutoPlay = false;

//-- this is the LED backpack
Adafruit_7segment matrix = Adafruit_7segment();

//-- capacitance touch sensor
Adafruit_MPR121 cap = Adafruit_MPR121();


#define VOutPin (12)
#define autoPlaySwitchPin (11)
#define GndPin (10)

//-- these are the output pins 2-13, which will correspond to the touch sensor
//#define startPinOut (2)
//#define numSensors (12)


#define NUM_FILTERED_SAMPLES (10)   // the number of samples that we'll be using

const int startDOUTPositivePin = 31;       // these will be numbered 31, 33, 35, 37, 39
const int startDOUTGroundPin = 30;         // these will be numbered 30, 32, 34, 36, 38
#define numSensors (12)

MSTimer playTimers[numSensors] = MSTimer();
boolean isPlaying[numSensors];

void setup() {


//-- Digital out pins
  // All ground pins
  for( int i = 0; i < numSensors; i++ ){
    pinMode(startDOUTGroundPin + (i*2), OUTPUT);
    digitalWrite(startDOUTGroundPin, LOW);
  }

  
  for( int i = 0; i < numSensors; i++) {
    pinMode(startDOUTPositivePin + (i*2), OUTPUT);
    digitalWrite(startDOUTPositivePin, LOW);
  }

   //-- for autoplay
   pinMode(VOutPin, OUTPUT);
   digitalWrite(VOutPin, HIGH);
   pinMode(GndPin, INPUT);
   digitalWrite(GndPin, LOW);
   pinMode(autoPlaySwitchPin, INPUT);
   
   
  Serial.begin(115200);
  Serial.println("starting up");

  //-- isPlaying = false for all
  for( int i; i < numSensors; i++ ) 
    isPlaying[i] = false;
    
  //-- initalize the 7-segment LED
  matrix.begin(0x70);
  matrix.print(9999, DEC);
  matrix.writeDisplay();

   Serial.println("matix init");
    
  // Use 0x5A for 5V from Arduino, 0x5B for 3.3V
  // try to init the capacitance sensor, keep at it until it works, but likely you will have to reset the Arduino
  if (!cap.begin(0x5A)) {
    matrix.writeDisplay();
    delay(100);
    Serial.println("no cap");
  }
  
  // 0 = normal state (nothing pressed)
  matrix.print(0, DEC);
  matrix.writeDisplay();

  Serial.println("running");
}

//-- get the touched status, which is a bitmask, i.e. pin 0 = 1 (if touched), pin 1 = 2 =
void loop() {
  checkAutoPlaySwitch();
  
  if( bAutoPlay )
    autoPlayMode();
  else
    interactiveMode();
}


//-- this is a digital in at pin 11, 5V is on pin 12, so bridge the two with jumpers
void checkAutoPlaySwitch() {
  //-- do debounce etc
  if( digitalRead(autoPlaySwitchPin) == true )
    bAutoPlay = true;
  else
    bAutoPlay = false;

  if( bAutoPlay ) 
    Serial.println("autoplay");
  else
    Serial.println("interactive");  
}

void autoPlayMode() {
  for( int i = 0; i < numSensors; i++ ) {
    // Check to see if we are playing and the playTimer is expired, if so, turn off
    if( isPlaying[i] == false ) {
      //--
      if( random(100000) < 5) {
        digitalWrite(startDOUTPositivePin + (i*2),HIGH);
        playTimers[i].setTimer( random(500, 5000) );
        isPlaying[i] = true;
      }
      
    }
    else if( isPlaying[i] == true && playTimers[i].isExpired() ) {
        digitalWrite(startDOUTPositivePin + (i*2),LOW);
        isPlaying[i] = false;
    }
  }
}

void interactiveMode() {
  //-- store in a local var for legibility
  uint16_t capBitmask = cap.touched();

//   matrix.begin(0x70);
//   matrix.print(capBitmask, DEC);
//   matrix.writeDisplay();
//
//  // delay(250);

 
/*
Serial.print("cap mask: " );
Serial.println(capBitmask);
*/
    
  //-- do cap-filtering here
   for(int i = 0; i < numSensors; i++ ) {
      //-- bitshift operators are not working with Arduino, so we are fudging it
      uint16_t bitMask = 0;
    
      //-- bitmask hack because >> isn't working
      bitMask = 1;
      for( int j = 0; j < i; j++ )
        bitMask = bitMask * 2;
      
      for( int j = 0; j < NUM_FILTERED_SAMPLES; j++ ) {
        if( capBitmask & bitMask ) 
          digitalWrite(startDOUTPositivePin + (i*2),HIGH);
        else
          digitalWrite(startDOUTPositivePin + (i*2),LOW);
      }
   }
}
