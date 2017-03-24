#include "Ultrasonic.h"
#include <Servo.h>

//Blue servo = pin 2
// Black servo = pin 4
// Ping Upper = Pin 3,5
//Ping lower = Pin 6,7


//
//Trigger,Echo
//Ultrasonic ultrasonicTop(3, 5);
//Ultrasonic ultrasonicBottom(6, 7);
//long range[7];
boolean checkUpper;
boolean canHit;
int pos;
Servo hitServo;
const int RIGHT_BACKWARD = 4;
const int RIGHT_FORWARD = 5;
const int LEFT_FORWARD = 6;
const int LEFT_BACKWARD = 7;
 //4 (Yellow) Right - Backwards 
  //5 (Red) Right - Forwards
  // 6 (Blue) Left - Forwards
  // 7 (Green) Left - Backwards


void setup() {
  Serial.begin(9600);
  
  pos = 0;
  hitServo.attach(4);
  //hitBall();
  pinMode(RIGHT_BACKWARD,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACKWARD,OUTPUT);
  delay(10);
  //drive();

}

void loop() {

//  // Drive Forward
//  boolean checkUpper = false;
//  boolean canHit = true;
//
//  check();
//  if (canHit) {
//    hitBall();
//  }
//  drive();
//  delay(1000);

 testDir();
}


//
//void check() {
//  long distanceBottom = (ultrasonicBottom.Ranging(CM));
//  Serial.print(distanceBottom);
//  Serial.println("cm ");
//  if (distanceBottom >= 2 && distanceBottom <= 6) {
//    checkUpper = true;
//  }
//  if ( checkUpper) {
//    long distanceTop = (ultrasonicTop.Ranging(CM));
//
//    for (int i = 0; i < 7; i ++) {
//      range[i] = i + 3 + (distanceTop % 3 );
//    }
//    for (int x = 0 ; x < 7; x++) {
//      if (range[x] == distanceBottom) {
//        canHit = false;
//      }
//    }
//  }
//}
//
//void hitBall(){
//  for (pos = 30; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    hitServo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(1);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    hitServo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(1);                       // waits 15ms for the servo to reach the position
//  }
//}
//
//void drive(){
//  analogWrite(bl,100);
//  analogWrite(fr,100);
//  delay(3000);
//  digitalWrite(fl,LOW);
//  digitalWrite(fr,LOW);
//}

void testDir(){
  Serial.println("4");
  digitalWrite(7,HIGH);
  analogWrite(7,200);
  delay(1000);
  digitalWrite(7,LOW);
  delay(1000);

  //4 (Yellow) Right - Backwards 
  //5 (Red) Right - Forwards
  // 6 (Blue) Left - Forwards
  // 7 (Green) Left - Backwards
}

void driveForward(){
	Serial.println("MOVING FORWARD") ; 
	digitalWrite(RIGHT_FORWARD, HIGH); 
	digitalWrite(LEFT_FORWARD, HIGH) ; 
	analogWrite (RIGHT_FORWARD, 100) ; 
	analogWrite (LEFT_FORWARD, 100) ;
	delay(100);
	digitalWrite(RIGHT_FORWARD, LOW);
	digitalWrite(LEFT_FORWARD, LOW) ;





