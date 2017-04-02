#include "Ultrasonic.h"
#include <Servo.h>
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>


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
 

  /* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);


void setup() {
  Serial.begin(9600);
  
  pos = 0;
  hitServo.attach(4);
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
  uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
}




