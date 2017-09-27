#include <SPI.h>
//#include <printf.h>
#include "RF24.h"
#include "LowPower.h"

RF24 radio(8,10);                        // Set up nRF24L01 radio on SPI bus plus pins

#define MOT1A    3
#define MOT1B    5
#define MOT2A    6
#define MOT2B    9
#define FrontLED    4
#define SleepTimeout    60000
#define StopTimeout    1000

unsigned long PreviousTime;

const uint64_t pipe =  0xE8E8F0F0E1LL;

typedef struct {
  int carspeed;
  int steering;
} CarData;

CarData data;

void setup(void) {
  pinMode(MOT1A, OUTPUT);
  digitalWrite(MOT1A, LOW);
  pinMode(MOT1B, OUTPUT);
  digitalWrite(MOT1B, LOW);
  pinMode(MOT2A, OUTPUT);
  digitalWrite(MOT2A, LOW);
  pinMode(MOT2B, OUTPUT);
  digitalWrite(MOT2B, LOW);
  pinMode(FrontLED, OUTPUT);
  digitalWrite(FrontLED, HIGH);

  Serial.begin(115200);
//  printf_begin();

  radio.begin();                           // Setup and configure rf radio
  radio.openReadingPipe(1,pipe);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
//  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  
  Serial.println(F("\n\rRF24/examples/Receive/"));
  Serial.println("Waiting for data");
  data.carspeed = 0;
  data.steering = 0;
}

void loop(void){
  if ( radio.available() )
  {
    radio.read( &data, sizeof(data) );
    Serial.println("Received data");
    Serial.println(data.carspeed,DEC);
    Serial.println(data.steering,DEC);
    SpeedMotor(data.carspeed);
    SteeringMotor(data.steering);
    PreviousTime = millis();
  }
  Serial.print("Previous: ");
  Serial.println(PreviousTime);
  Serial.print("Current: ");
  Serial.println(millis());
  
  if (millis() > PreviousTime + StopTimeout) {
    SpeedMotor(0);
    SteeringMotor(0);
  }

  if (millis() > PreviousTime + SleepTimeout) {
    radio.stopListening();
    radio.powerDown();
    digitalWrite(FrontLED, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

void SpeedMotor(int MotorValue) {
  int MotorPin1;
  int MotorPin2;
  if (MotorValue < 0) {
    MotorPin1 = MOT1A;
    MotorPin2 = MOT1B;
  } else {
    MotorPin1 = MOT1B;
    MotorPin2 = MOT1A;
  }
  analogWrite(MotorPin1, abs(MotorValue));
  analogWrite(MotorPin2, 0);
}

void SteeringMotor(int MotorValue) {
  int MotorPin1;
  int MotorPin2;
  if (MotorValue < 0) {
    MotorPin1 = MOT2A;
    MotorPin2 = MOT2B;
  } else {
    MotorPin1 = MOT2B;
    MotorPin2 = MOT2A;
  }
  analogWrite(MotorPin1, abs(MotorValue));
  analogWrite(MotorPin2, 0);
}

