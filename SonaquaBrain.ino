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

#define LED_PIN (13)

//-- these are the output pins 2-13, which will correspond to the touch sensor
//#define startPinOut (2)
//#define numSensors (12)


#define NUM_FILTERED_SAMPLES (10)   // the number of samples that we'll be using

const int startDOUTPositivePin = 31;       // these will be numbered 31, 33, 35, 37, 39
const int startDOUTGroundPin = 30;         // these will be numbered 30, 32, 34, 36, 38
#define numSensors (12)

MSTimer playTimers[numSensors] = MSTimer();

void setup() {
  //-- initalize digital pins 2-13 for output
  /*
   for( int i = 0; i < numSensors; i++ ) {
    pinMode(startPinOut + i,OUTPUT);
  }
  */

/*
  for( int i = 0; i < numSensors; i++ ) {
    digitalWrite(startPinOut + i,HIGH);
     delay(100);
    digitalWrite(startPinOut + i,LOW);
    delay(100);
  }
 */
 
  //-- led pin (are we going to use this?)
  pinMode(LED_PIN,OUTPUT);

  for( int i = 0; i < 4;i ++ ) {
    digitalWrite(LED_PIN,HIGH);
    delay(100);
    digitalWrite(LED_PIN,LOW);
    delay(100);
  }

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

//-- TESTER for pins 12 always OUT
   pinMode(12, OUTPUT);
   digitalWrite(12, HIGH);
  
  Serial.begin(115200);
  Serial.println("starting up");
  
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
  if( bAutoPlay )
    autoPlayMode();
  else
    interactiveMode();
}

void autoPlayMode() {
  for( int i = 0; i < numSensors; i++ ) {
//    if( playTimers[i].isExpired() ) {
//      
//      
//    }
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

 //-- NOT SURE WHY BUT BITMASK
 /*
  unsigned short bitShift = 1;
  Serial.println("-------------");
   Serial.println(capBitmask);
   //Serial.println( capBitmask & (1) );
   //Serial.println( capBitmask & (1 >> 0) );
    Serial.println(  (1 >> 0) );
     
   //Serial.println( capBitmask & (2) );
   //Serial.println( capBitmask & (1 >> 1) );
   bitShift = (1 >> 1);
   Serial.println( bitShift );
   //Serial.println( capBitmask & (4) );
   //Serial.println( capBitmask & (1>>2) );
    bitShift = (1 >> 2);
   Serial.println(  bitShift  );
      Serial.println("-------------");
*/

Serial.print("cap mask: " );
Serial.println(capBitmask);

    
  //-- do cap-filtering here
   for(int i = 0; i < numSensors; i++ ) {
      //-- bitshift operators are not working with Arduino, so we are fudging it
      uint16_t bitMask = 0;
     /* if( i == 0 )
        bitMask = 1;
      else {
       */
        
      //}
       
       
       //bitMask = pow(2, i);

      bitMask = 1;
      for( int j = 0; j < i; j++ )
        bitMask = bitMask * 2;
          
   //       Serial.print("bitMask: " );
   //     Serial.println(bitMask);
      
      for( int j = 0; j < NUM_FILTERED_SAMPLES; j++ ) {
        if( capBitmask & bitMask ) 
          digitalWrite(startDOUTPositivePin + (i*2),HIGH);
        else
          digitalWrite(startDOUTPositivePin + (i*2),LOW);
      }
   }
}
