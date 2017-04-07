#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include "Ultrasonic.h"
#include <Servo.h>
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//TODO: Rotate Hitter
//TODO: Hit ball

struct Color {
  uint16_t r = 0; 
  uint16_t g = 0;
  uint16_t b = 0;
  uint16_t c = 0;
} color;

/*
 * Motor init
 */

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree) to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

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

/*
 * Globals
 */
 //TODO: tune constants
int MOVE = 500; //delay for moving (delay for half second)
int STEP5 = 50; //delay scale for rotation (delay time to move 5 degrees)

//servo vars
Servo myservo;
int pos = 0;

//ball detection
long distanceTop;
long distanceBottom;

//movement track
int ballRot = 0; //rotation from line to find ball



/*
 * START
 */

void setup() {
  Serial.begin(9600);

  myMotor->setSpeed(50);  // 10 rpm  
  
  myservo.attach(12);  // attaches the servo on pin 9 to the servo object
//  Serial.println("ATTACH DIS BITCH");
  //hitBall();
  pinMode(RIGHT_BACKWARD,OUTPUT);
  pinMode(RIGHT_FORWARD,OUTPUT);
  pinMode(LEFT_FORWARD,OUTPUT);
  pinMode(LEFT_BACKWARD,OUTPUT);
  
  
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
// printColor();
 delay(1000);
}


//
void check() {
  long distanceBottom = (ultrasonicBottom.distanceRead());
  Serial.print("BOTTOM: ");
  Serial.print(distanceBottom);
  Serial.println(" CM");
  long distanceTop = (ultrasonicTop.distanceRead());
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
  move();	
}

void printColor(){
  uint16_t colorTemp, lux;
  Color curColor;
  tcs.getRawData(&curColor.r, &curColor.g, &curColor.b, &curColor.c);
 long avg = curColor.c/3 ;
 curColor.r - curColor.r/avg;
 curColor.g - curColor.g/avg;
 curColor.b - curColor.b/avg;
  Serial.print("R: "); Serial.print(curColor.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(curColor.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(curColor.b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(curColor.c, DEC); Serial.print(" ");
  Serial.println(" ");
}

int isRed(Color &curColor){
  return (curColor.r > 2000 && curColor.g < 100 && curColor.b < 100);
}

void move() {
  for (pos = 0; pos <= 180; pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Serial.println("HEER");
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(100);                       // waits 15ms for the servo to reach the position
  }
  
}

/*
 * ------- BASIC Movement --------
 */
 
/**
 * deg = degrees to turn
 * dist = distance to move (
 */
 void drive(int deg, int dist) {
  rotate(deg);
  go(dist);
 }


void rotate(int deg){
  //rotate (POS = clockwise, NEG = counter-clockwise)
  Serial.print("Rotating: "); 
  Serial.println(deg);

  if (deg > 0 && deg <= 360) {
    /*
     * H   |
     * |---|
     * |   H
     */
    digitalWrite(RIGHT_FORWARD, LOW); 
    digitalWrite(RIGHT_BACKWARD, HIGH); 
    digitalWrite(LEFT_FORWARD, HIGH); 
    digitalWrite(LEFT_BACKWARD, LOW);

    analogWrite (RIGHT_FORWARD, 150); 
    analogWrite (RIGHT_BACKWARD, 0);
    analogWrite (LEFT_FORWARD, 150);
    analogWrite (LEFT_BACKWARD, 0);     
  } else if (deg < 0 && deg > -360) {
    /*
     * |   H
     * |---|
     * H   |
     */
    digitalWrite(RIGHT_FORWARD, HIGH); 
    digitalWrite(RIGHT_BACKWARD, LOW); 
    digitalWrite(LEFT_FORWARD, LOW); 
    digitalWrite(LEFT_BACKWARD, HIGH);

    analogWrite (RIGHT_FORWARD, 150); 
    analogWrite (RIGHT_BACKWARD, 0);
    analogWrite (LEFT_FORWARD, 150);
    analogWrite (LEFT_BACKWARD, 0);    
  }

  delay(STEP5 * (deg/5));
  
  analogWrite (RIGHT_FORWARD, 0); 
  analogWrite (RIGHT_BACKWARD, 0);
  analogWrite (LEFT_FORWARD, 0);  
  analogWrite (LEFT_BACKWARD, 0);
  digitalWrite(RIGHT_FORWARD, LOW); 
  digitalWrite(RIGHT_BACKWARD, LOW); 
  digitalWrite(LEFT_FORWARD, LOW); 
  digitalWrite(LEFT_BACKWARD, LOW);
}

void go(int dist) {
  if (dist) {  
    digitalWrite(RIGHT_FORWARD, HIGH); 
    digitalWrite(LEFT_FORWARD, HIGH); 
    digitalWrite(RIGHT_BACKWARD, LOW); 
    digitalWrite(LEFT_BACKWARD, LOW);
    analogWrite (RIGHT_BACKWARD, 0); 
    analogWrite (LEFT_BACKWARD, 0);
    analogWrite (RIGHT_FORWARD, 150);
    analogWrite (LEFT_FORWARD, 150);
    
    delay(MOVE * dist);
   
    analogWrite (RIGHT_FORWARD, 0); 
    analogWrite (RIGHT_BACKWARD, 0);
    analogWrite (LEFT_FORWARD, 0);  
    analogWrite (LEFT_BACKWARD, 0);
    digitalWrite(RIGHT_FORWARD, LOW); 
    digitalWrite(RIGHT_BACKWARD, LOW); 
    digitalWrite(LEFT_FORWARD, LOW); 
    digitalWrite(LEFT_BACKWARD, LOW);
  }
}

/*
 * ------- ADV Movement -------
 */

 //TODO: move along line

/*
 * Ball Detection
 */

// Return 1 if found in 360 around robot, else 0
int findBall(){
  int found = seeBall();
  ballRot = 0;
  
  //rotate while checking ball  
  while(!found || ballRot == 360){
    drive(5, 0);
    ballRot += 5;
    found = seeBall();
  }

  if (found && ballRot < 360) {
    return 1;   
  } else {
    return 0;
  }
}

//return 1 if radar input sees ball, else 0
int seeBall(){
  distanceTop = (ultrasonicTop.distanceRead());
  distanceBottom = (ultrasonicBottom.distanceRead());
  printRadar();

  if (distanceTop - distanceBottom >= 10) {
    return 1;
  } else {
    return 0;
  }
}

void printRadar() {
  Serial.print("TOP: ");
  Serial.print(distanceTop);
  Serial.println(" CM");
  Serial.print("BOTTOM: ");
  Serial.print(distanceBottom);
  Serial.println(" CM");

  if (distanceTop - distanceBottom >= 10) {
    Serial.println("---TARGET DETECTED---");
  }  
}
 



