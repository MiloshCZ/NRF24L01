/*
TMRh20 2014

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/** General Data Transfer Rate Test
 * This example demonstrates basic data transfer functionality with the 
 updated library. This example will display the transfer rates acheived using
 the slower form of high-speed transfer using blocking-writes.
 */


#include <SPI.h>,
//#include <printf.h>
#include "RF24.h"
#include "LowPower.h"

/*************  USER Configuration *****************************/
                                          // Hardware configuration
RF24 radio(9,10);                        // Set up nRF24L01 radio on SPI bus plus pins 7 & 8

/***************************************************************/

const uint64_t pipe = 0xE8E8F0F0E1LL;

typedef struct {
  int carspeed;
  int steering;
} CarData;

CarData data;

#define PotX  A6
#define PotY  A7
#define PotVcc  8
#define PotSW  3
#define ThrottleOffset -4
#define ThrottleLimit 255
#define ThrottleMultiplier 0.5
#define ThrottleMin 10
#define SteeringOffset 0
#define SteeringLimit 255
#define SteeringMultiplier 2
#define SteeringMin 10

#define SleepTimeout    60000

unsigned long PreviousTime;

void setup(void) {

  pinMode(PotVcc, OUTPUT);
  digitalWrite(PotVcc, HIGH);
  pinMode(PotSW, INPUT_PULLUP);
  analogReference(DEFAULT);
  
  Serial.begin(115200);
//  printf_begin();

  radio.begin();                           // Setup and configure rf radio
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
 
//  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  
  Serial.println(F("\n\rRF24/examples/Transfer/"));
  
  randomSeed(analogRead(0));              //Seed for random number generation
  
  radio.powerUp();                        //Power up the radio
}

void loop(void){
  float Throttle = constrain((256 - (analogRead(PotX) >> 1) - ThrottleOffset) * ThrottleMultiplier, -ThrottleLimit, ThrottleLimit);
  float Steering = constrain((256 - (analogRead(PotY) >> 1) - SteeringOffset) * SteeringMultiplier, -SteeringLimit, SteeringLimit);
  Serial.println((int)Throttle,DEC);
  Serial.println((int)Steering,DEC);
  
  if ( (abs(Throttle) > ThrottleMin) || (abs(Steering) > SteeringMin) ) {
    PreviousTime = millis();
  }
  radio.powerUp();
  radio.stopListening();
  data.carspeed = (int)Throttle;
  data.steering = (int)Steering;
  if (!radio.write(&data,sizeof(data))){
     Serial.println("Chyba při odeslání!");
  }
  radio.startListening();
  radio.powerDown();
  if (millis() > PreviousTime + SleepTimeout) {
    Serial.println("Sleep");
    delay(500);
    radio.stopListening();
    radio.powerDown();
    digitalWrite(PotVcc, LOW);
    attachInterrupt(digitalPinToInterrupt(PotSW), PotSW, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(0);
    delay(500);
    Serial.println("WakeUp");
    digitalWrite(PotVcc, HIGH);
  }
}
