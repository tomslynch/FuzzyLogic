#include "Ultrasonic.h"
#include <Servo.h>

//Blue servo = pin 2
// Black servo = pin 4
// Ping Upper = Pin 3,5
//Ping lower = Pin 6,7


//
//Trigger,Echo
Ultrasonic ultrasonicTop(3, 5);
Ultrasonic ultrasonicBottom(6, 7);
long range[7];
boolean checkUpper;
boolean canHit;
int pos;
Servo hitServo;
const int fl = 9;
const int bl = 10;
const int fr = 11;
const int br = 12;



void setup() {
  Serial.begin(9600);
  
  pos = 0;
  hitServo.attach(4);
  hitBall();
  pinMode(fl,OUTPUT);
  pinMode(fr,OUTPUT);
  pinMode(bl,OUTPUT);
  pinMode(br,OUTPUT);
  delay(10);
  drive();

}

void loop() {

  // Drive Forward
  boolean checkUpper = false;
  boolean canHit = true;

  check();
  if (canHit) {
    hitBall();
  }
  drive();
  delay(1000);
}



void check() {
  long distanceBottom = (ultrasonicBottom.Ranging(CM));
  Serial.print(distanceBottom);
  Serial.println("cm ");
  if (distanceBottom >= 2 && distanceBottom <= 6) {
    checkUpper = true;
  }
  if ( checkUpper) {
    long distanceTop = (ultrasonicTop.Ranging(CM));

    for (int i = 0; i < 7; i ++) {
      range[i] = i + 3 + (distanceTop % 3 );
    }
    for (int x = 0 ; x < 7; x++) {
      if (range[x] == distanceBottom) {
        canHit = false;
      }
    }
  }
}

void hitBall(){
  for (pos = 30; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    hitServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    hitServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
}

void drive(){
  analogWrite(bl,100);
  analogWrite(fr,100);
  delay(3000);
  digitalWrite(fl,LOW);
  digitalWrite(fr,LOW);
}





