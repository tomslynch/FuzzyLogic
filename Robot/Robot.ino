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
 * CONSTANTS
 */
//TODO: tune constants
int MOVE = 500; //delay for moving (delay for half second)
int STEP5 = 50; //delay scale for rotation (delay time to move 5 degrees)

int DIST_UNIT = 1;  //movement unit
int ROT_UNIT = 5;   //degree unit (5 degrees)

int DISTANCE_LIMIT = 10;    //radar variation

int COLOR_HIGH = 2000;
int COLOR_LOW = 100;

int CHASSIS_DIS = 1;

int PLAY_SCALE = 10;
int RADAR_SCALE = 10;

int HITTING_DIS = 10;
int FAILSAFE_LIMIT = 15;

int FASTHIT_POS = 120;
int SLOWHIT_POS = 90;


/*
 * Globals
 */

// color vars
Color curColor;

//servo vars
Servo myservo;
int pos = 0;

//ball detection
long distanceTop;
long distanceBottom;

long lastSeen;

//movement track
int ballRot = 0; //rotation from line to find ball
int redRot = 0;

//scan mode
int scanDeg[10];
int scanDis[10];
int lineCounter = 0;
int lineTotal = 0;

//play mode
int playDeg[10];
int playDis[10];

//tether
int tetherDeg[10];
int tetherDis[10];
int tetherCounter;

//hitter
int strokes = 0;

//failsafe
long randDeg = 0;
long randDis = 0;

//mode button
//TODO



/*
 * -------- START --------
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

    randomSeed(analogRead(0));  //for failsafe

}

void loop() {
    //TODO: check for modes with button
    strokes = 20;
    drive(250,250);
//dance();
//  play();
}

/**
 * -------- SENSOR FUNCTIONS --------
 */

/*
 * Color Detection
 */

void updateColor(){
  tcs.getRawData(&curColor.r, &curColor.g, &curColor.b, &curColor.c);
  long avg = curColor.c/3 ;
  curColor.r = curColor.r/avg;
  curColor.g = curColor.g/avg;
  curColor.b = curColor.b/avg;
  
  Serial.print("R: "); Serial.print(curColor.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(curColor.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(curColor.b, DEC); Serial.print(" ");
  Serial.println(" ");
}
// 1 if sees red, else 0
int isRed(){
  updateColor();
  if (curColor.r > COLOR_HIGH && curColor.g < COLOR_LOW && curColor.b < COLOR_LOW) {
    return 1;
  } else {
    return 0;
  }
}


/*
 * Ball Detection
 */

// Return 1 if found in 360 around robot, else 0
int findBall(){
  int found = seeBall();
  ballRot = 0;
  
  //rotate while checking ball  
  while(!found || ballRot == 360){
    rotateOne();
    ballRot += ROT_UNIT;
    found = seeBall();
      printStatus("    {BALL}", "SEARCHING...", ballRot);
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

  if (distanceTop - distanceBottom >= DISTANCE_LIMIT) {
      lastSeen = distanceBottom;
    return 1;
  } else {
    return 0;
  }
}
void printRadar() {
    printStatus("    {BALL}", "   TOP: ", distanceTop);
    printStatus("    {BALL}", "   BOTTOM: ", distanceBottom);

  if (distanceTop - distanceBottom >= 10) {
    Serial.println("---TARGET DETECTED---");
  }  
}


/**
 * ------- BASIC Movement --------
 */
 
/*
 * deg = degrees to turn
 * dist = distance to move
 */
 void drive(int deg, int dist) {
  rotate(deg);
  go(dist);
 }

void rotate(int deg){
  //rotate (POS = clockwise, NEG = counter-clockwise)
    printStatus("{MOVEMENT} ", "Rotating: ", deg);

  if (deg > 0 && deg <= 360) {
    /* -> clockwise ->
     * H   |
     * |---|
     * |   H
     */
    digitalWrite(RIGHT_FORWARD, LOW); 
    digitalWrite(RIGHT_BACKWARD, HIGH); 
    digitalWrite(LEFT_FORWARD, HIGH); 
    digitalWrite(LEFT_BACKWARD, LOW);

    analogWrite (RIGHT_FORWARD, 0); 
    analogWrite (RIGHT_BACKWARD, 150);
    analogWrite (LEFT_FORWARD, 150);
    analogWrite (LEFT_BACKWARD, 0);     
  } else if (deg < 0 && deg > -360) {
    /* <- counter-clockwise <-
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
    analogWrite (LEFT_FORWARD, 0);
    analogWrite (LEFT_BACKWARD, 150);    
  }

  delay(STEP5 * (deg/5));
 
  resetTracks();
}
void go(int dist) {
    printStatus("{MOVEMENT} ", "DRIVING: ", dist);
  if (dist > 0) {  
    //Forward
    digitalWrite(RIGHT_FORWARD, HIGH); 
    digitalWrite(RIGHT_BACKWARD, LOW); 
    digitalWrite(LEFT_FORWARD, HIGH); 
    digitalWrite(LEFT_BACKWARD, LOW);

    analogWrite (RIGHT_FORWARD, 150);
    analogWrite (RIGHT_BACKWARD, 0); 
    analogWrite (LEFT_FORWARD, 150);
    analogWrite (LEFT_BACKWARD, 0);
    
    delay(MOVE * dist);
   
  } else if (dist < 0) {
    //Reverse
      digitalWrite(RIGHT_FORWARD, LOW);
      digitalWrite(RIGHT_BACKWARD, HIGH);
      digitalWrite(LEFT_FORWARD, LOW);
      digitalWrite(LEFT_BACKWARD, HIGH);

      analogWrite (RIGHT_FORWARD, 0);
      analogWrite (RIGHT_BACKWARD, 150);
      analogWrite (LEFT_FORWARD, 0);
      analogWrite (LEFT_BACKWARD, 150);

      delay(MOVE * abs(dist));
  }

  resetTracks();
}
void resetTracks() {
    analogWrite (RIGHT_FORWARD, 0); 
    analogWrite (RIGHT_BACKWARD, 0);
    analogWrite (LEFT_FORWARD, 0);  
    analogWrite (LEFT_BACKWARD, 0);
    digitalWrite(RIGHT_FORWARD, LOW); 
    digitalWrite(RIGHT_BACKWARD, LOW); 
    digitalWrite(LEFT_FORWARD, LOW); 
    digitalWrite(LEFT_BACKWARD, LOW); 
}

void stepOne() {
    drive(0, 5);
}
void rotateOne() {
    drive (5, 0);
}

/**
 * -------- MODES --------
 */
 
/**
 * Pathfinder MODE
 */

 void pathfinder() {
  while(findRed()){
      recordLine();
      printLine();
      lineCounter++;
  }
    printStatus("    {PATH}", "END PATHFINDER", redRot);
 }

//Color sensor is on rear of chassis
//return 1 if found red, if not return 0
int findRed() {  
  int found = isRed();
  redRot = 0;

    //first rotate 180 for rear of chassis to get readings
    drive(180, 0);

  //Rotate to find red
  while(redRot != 350 || !found){
    rotateOne();
    redRot += ROT_UNIT;
    found = isRed();
      printStatus("    {PATH}", "SEARCHING...", redRot);
  }

  if (found && redRot < 350) {
      scanDeg[lineCounter] = redRot;
    return 1;   
  } else {
    return 0;
  }
}
void recordLine() {
    int curDistance = 0;

    //go along line until white
    while(isRed()) {
        stepOne();
        curDistance += DIST_UNIT;
        printStatus("    {PATH}", "MOVING...", curDistance);
    }

    //Not red! back up chassis length
    go(-CHASSIS_DIS);

    //record
    scanDis[lineCounter] = curDistance;
}
void printLine() {
    Serial.print("[");
    Serial.print(lineCounter);
    Serial.print("]-degree: ");
    Serial.print(scanDeg[lineCounter]);
    Serial.print(", distance: ");
    Serial.println(scanDis[lineCounter]);
}

/**
 * Play MODE
 */

 void play(){
    //pull values from pathfinder results
    scalePathfinder();

     //Hit ball from start position
     HITTING(playDeg[lineCounter], playDis[lineCounter]);

     while(lineCounter != lineTotal || strokes >= FAILSAFE_LIMIT) {
         tetherCounter = 0;

         //move along line (half distance)
         drive(playDeg[lineCounter], playDis[lineCounter]/2);
         playDis[lineCounter] -= playDis[lineCounter]/2;

         //Scan for ball
         if (!findBall()) {
             drive(0, playDis[lineCounter]);
             playDis[lineCounter] = 0;
         }

         //TODO: handle worst case scenarios - can't find after moving across entire line
         if (findBall()) {
             //tether to correct ball
             tetherDeg[0] = ballRot - ROT_UNIT;              // offset robot
             tetherDis[0] = (lastSeen-10)/RADAR_SCALE;       // scale down distance
             tether();
         } else {
             for (int i = 0; i < 10000; i++) {
                 Serial.println("DOES NOT COMPUTE");
             }
         }

         //decrement distance left at index position
         if (playDis[lineCounter] != 0) {
             drive(0, tetherDis[lineCounter]);
         }

         lineCounter++;
     }

    if (strokes >= FAILSAFE_LIMIT) {
        godspeed();
    }
}

void scalePathfinder() {
    lineTotal = lineCounter;
    lineCounter = 0;
    Serial.println("Your knowledge belongs to me, Pathfinder.");
    for (int i = 0; i <= lineTotal; i++) {
        playDeg[i] = scanDeg[i];
        playDis[i] = scanDis[i] * PLAY_SCALE;
    }
    Serial.println("Your knowledge has been consumed.");
}


void tether() {
    int distance = 0;
    int returnDeg = 0;

    printStatus("    {TETHER}", "DISEMBARKING... Distance: ", tetherDis[0]);
    //Get to ball
    drive(-ROT_UNIT, tetherDis[tetherCounter]);

    //find ball
    findBall();

    //record rotation and distance spotted
    tetherCounter++;
    tetherDeg[tetherCounter] = ballRot;

    //quick check if ball is really close to us
    if (lastSeen < HITTING_DIS) {
        drive(0, -DIST_UNIT);
    }

    //move towards ball until in hitting range
    while (seeBall() && lastSeen > HITTING_DIS) {
        printStatus("    {TETHER}", "MOVING TOWARDS BALL... Distance: ", DIST_UNIT);
        drive(0, DIST_UNIT);
        distance += DIST_UNIT;
    }

    tetherDis[tetherCounter] = distance;

    //Should be in range to hit now
    //TODO: Calculate angle and distance with triangles?
    if (playDis[lineCounter] != 0) {
        if (tetherDeg[0] > 180) {
            returnDeg = (tetherDeg[0]/2);
        } else {
            returnDeg = (-tetherDeg[0]/2);
        }
    }
    HITTING(returnDeg, tetherDis[0]);

    //Reverse to line
    returnToLine();
}

void returnToLine() {
    for (int i = tetherCounter; i >= 0; i--) {
        printStatus("    {TETHER}", "RETURNING TO LINE... ", tetherCounter);
        drive(0, -tetherDis[i]);
        drive(-tetherDeg[i], 0);
    }
}

//failsafe methods
void godspeed() {
    dance();

    int deg = 0;
    int dis = 0;

    failsafeSpam();
    while(1) {
        while(findBall()){
            while(seeBall() && lastSeen > HITTING_DIS) {
                drive(0, DIST_UNIT);
            }
            HITTING(45, 3);
        }

        randDeg = random(300);
        randDis = random(5);
        drive(randDeg, randDis);
    }
}
void failsafeSpam() {
    printStatus("-------------------", "WARNING! -------------------", -1);
    printStatus("---------------", "ПРЕДУПРЕЖДЕНИЕ! ----------------", -1);
    printStatus("-------------------", "WARNUNG! -------------------", -1);
    printStatus("------------------", "VAROVÁNÍ! -------------------", -1);
    printStatus("---------------------", "警告! --------------------", -1);
    printStatus("-----------------", "ADVERTENCIA! -----------------", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "   FAILSAFE ENGAGED   XXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("XXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX", -1);
    printStatus("-------------------", "WARNING! -------------------", -1);
    printStatus("---------------", "ПРЕДУПРЕЖДЕНИЕ! ----------------", -1);
    printStatus("-------------------", "WARNUNG! -------------------", -1);
    printStatus("------------------", "VAROVÁNÍ! -------------------", -1);
    printStatus("---------------------", "警告! --------------------", -1);
    printStatus("-----------------", "ADVERTENCIA! -----------------", -1);
}
void dance() {
    drive(100, 0);
    drive(-100, 0);
    drive(100, 0);
    drive(-100, 0);
}

/**
 * HITTING Functions
 */

void HITTING (int deg, int dis) {
    int stepperDeg = (deg/10)*2;   //TODO: figure out how to make this close to being accurate
    //move hitter into position     //TODO: Change stepperDeg val
//    myMotor.step(stepperDeg); //TODO

    printStatus("    {HITTING}", "HITTING BALL Degree: ", stepperDeg);
    printStatus("    {HITTING}", "HITTING BALL Distance: ", stepperDeg);
    //swing FAST or SLOW
    if (dis >= 3) {
        myservo.write(FASTHIT_POS);
        printStatus("    {HITTING}", "HITTING BALL (FAST)", -1);
    } else {
        myservo.write(SLOWHIT_POS);
        printStatus("    {HITTING}", "HITTING BALL (SLOW)", -1);
    }

    strokes++;
    resetHitter();
}

void resetHitter() {
    //rotate disc to default
    //rotate hitter to default
    //TODO
}


/**
 * Status Printer
 */

void printStatus(char * prefix, char * msg, int opt) {
    Serial.print(prefix);
    Serial.print(msg);
    if (opt != -1) {
        Serial.println(opt);
    } else {
        Serial.println();
    }
}





/**
 * Delete?
 */

//
//void driveForward(){
//	Serial.println("MOVING FORWARD") ;
//	digitalWrite(RIGHT_FORWARD, HIGH);
//	digitalWrite(LEFT_FORWARD, HIGH) ;
//  digitalWrite(RIGHT_BACKWARD, LOW);
//  digitalWrite(LEFT_BACKWARD, LOW) ;
//  analogWrite (RIGHT_BACKWARD, 0) ;
//  analogWrite (LEFT_BACKWARD, 0) ;
//	analogWrite (RIGHT_FORWARD, 150) ;
//	analogWrite (LEFT_FORWARD, 150) ;
//
//	delay(100);
//
//  analogWrite (RIGHT_FORWARD, 0) ;
//  analogWrite (LEFT_FORWARD, 0) ;
//  digitalWrite(RIGHT_FORWARD, LOW);
//  digitalWrite(LEFT_FORWARD, LOW) ;
//  move();
//}

//void move() {
//  for (pos = 0; pos <= 180; pos += 5) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    Serial.println("HEER");
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(100);                       // waits 15ms for the servo to reach the position
//  }
//
//}
 



