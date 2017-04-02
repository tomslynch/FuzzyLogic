#include "Ultrasonic.h"
#include <Servo.h>
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

struct Color {
  uint16_t r = 0; 
  uint16_t g = 0;
  uint16_t b = 0;
  uint16_t c = 0;
} color;

// Ping Upper = Pin 8,9
//Ping lower = Pin 10,11
//Trigger,Echo
Ultrasonic ultrasonicTop(8, 9);
Ultrasonic ultrasonicBottom(10, 11);


//4 (Yellow) Right - Backwards 
//5 (Red) Right - Forwards
// 6 (Blue) Left - Forwards
// 7 (Green) Left - Backwards
const int RIGHT_BACKWARD = 4;
const int RIGHT_FORWARD = 5;
const int LEFT_FORWARD = 6;
const int LEFT_BACKWARD = 7;

/* init color values */
Color prevColor;  //TODO: setup to check center later
 

  /* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600);
  

  //hitBall();
  pinMode(RIGHT_BACKWARD,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACKWARD,OUTPUT);
  Serial.begin(9600);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  delay(10);
  //drive();

}

void loop() {
 check();
 printColor();
 delay(1000);
}


//
void check() {
  long distanceBottom = (ultrasonicBottom.Ranging(CM));
  Serial.print("BOTTOM: ");
  Serial.print(distanceBottom);
  Serial.println(" CM");
  long distanceTop = (ultrasonicTop.Ranging(CM));
  Serial.print("TOP: ");
  Serial.print(distanceTop);
  Serial.println(" CM");
  if(distanceTop > distanceBottom){
    driveForward();
  }
}



void driveForward(){
	Serial.println("MOVING FORWARD") ; 
	digitalWrite(RIGHT_FORWARD, HIGH); 
	digitalWrite(LEFT_FORWARD, HIGH) ; 
  digitalWrite(RIGHT_BACKWARD, LOW); 
  digitalWrite(LEFT_BACKWARD, LOW) ;
  analogWrite (RIGHT_BACKWARD, 0) ; 
  analogWrite (LEFT_BACKWARD, 0) ;
	analogWrite (RIGHT_FORWARD, 150) ; 
	analogWrite (LEFT_FORWARD, 150) ;
	delay(100);
  analogWrite (RIGHT_FORWARD, 0) ; 
  analogWrite (LEFT_FORWARD, 0) ;
  digitalWrite(RIGHT_FORWARD, LOW); 
  digitalWrite(LEFT_FORWARD, LOW) ;
	
}

void printColor(){
  uint16_t colorTemp, lux;
  Color curColor;
  
  getColor(curColor);
  Serial.print("R: "); Serial.print(curColor.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(curColor.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(curColor.b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(curColor.c, DEC); Serial.print(" ");
  Serial.println(" ");
}

int isRed(Color &curColor){
  return (curColor.r > 200 && curColor.g < 100 && curColor.b < 100);
}

void getColor(Color curColor) {
  tcs.getRawData(&curColor.r, &curColor.g, &curColor.b, &curColor.c);
}



