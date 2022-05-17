#include <Servo.h>

const int dirPin = 2;
const int stepPin = 3;
const int maxRotation = 400; //stepsPerRevolution = 400 as we have a 1:2 gear ratio
const int proximitySensorPin = 5;
int proxVal;
int servoPos = 0;
int stepsPos = 0;
bool zeroPosTouched = false;
String debug = "Did not touch";
const int pRTopRightPin = A0; // Photoresistor at Arduino analog pin A0 BLUE - top right
const int pRTopLeftPin = A1; // Photoresistor at Arduino analog pin A1 GREEN - top left
const int pRBottomRightPin = A2; // Photoresistor at Arduino analog pin A0 GREY - bottom right
const int pRBottomLeftPin = A3; // Photoresistor at Arduino analog pin A1 PURPLE - bottom left
int topRightResVal;
int topLeftResVal;
int bottomRightResVal;
int bottomLeftResVal;
int photoOffset = 3;
Servo myservo;



void setup() {
  Serial.begin(9600);
  pinMode(proximitySensorPin, INPUT); 
  pinMode(dirPin, OUTPUT); 
  pinMode(stepPin,OUTPUT);
  digitalWrite(dirPin,HIGH);
  Serial.println("Moving 50 steps to make sure the startup havent moved the motor too far");
  for(int x = 0; x < 50; x++){
      digitalWrite(dirPin,HIGH);// HIGH = left (counterclockwise), LOW = right (clockwise) with black towards power of the motor
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
  }
  Serial.println("First steps done");
  delay(2000);
  // Test if the motor is already above the sensor, if so move it away and back to get the correct 0 position
  if (digitalRead(proximitySensorPin) == 0){
    Serial.println("Touched before moving, correting position");
    digitalWrite(dirPin,HIGH); //set direction and move it until it no longer touches the sensor
    while(digitalRead(proximitySensorPin) == 0){
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
    }
    for(int x = 0; x < 10; x++){ // move it 10 steps further to make sure its away from the sensor
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
    } // continue as if it never touched the sensor during setup
  }
  Serial.println("Touched before moving, correting position done");
  delay(2000);
  resetTo0();
  Serial.println("Calibration done, waiting 5 seconds before starting");
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(proximitySensorPin);

  topRightResVal = analogRead(pRTopRightPin);
  topLeftResVal = analogRead(pRTopLeftPin);
  bottomRightResVal = analogRead(pRBottomRightPin);
  bottomLeftResVal = analogRead(pRBottomLeftPin);
  //Serial.println("\n----------");
  //Serial.print("Top right:");
  //Serial.println(topRightResVal);
  //Serial.print("Top left:");
  //Serial.println(topLeftResVal);
  //Serial.print("Bottom right:");
  //Serial.println(bottomRightResVal);
  //Serial.print("Bottom left:");
  //Serial.println(bottomLeftResVal);
  if (topLeftResVal > (topRightResVal + photoOffset) || bottomLeftResVal > (bottomRightResVal + photoOffset)) {
    //move to the left
    Serial.println("Moving to the left");
    //left(5);
  }else if (topLeftResVal > (topRightResVal + photoOffset) || bottomLeftResVal > (bottomRightResVal + photoOffset)) {
    //move to the right
    Serial.println("Moving to the right");
    //right(5);
  }else if (topLeftResVal > (bottomLeftResVal + photoOffset) || topRightResVal > (topRightResVal + photoOffset)) {
    //move up
    Serial.println("Moving up");
  }else if (bottomLeftResVal > (topLeftResVal + photoOffset) || bottomRightResVal > (topRightResVal + photoOffset)) {
    //move down
    Serial.println("Moving down");
  }

  right(10);
  delay(500);
}

void resetTo0(){
  Serial.println("Resetting to 0 position");
  digitalWrite(dirPin,LOW);
  while(digitalRead(proximitySensorPin) != 0){
    digitalWrite(stepPin,HIGH);
    delay(5);
    digitalWrite(stepPin,LOW);
    delay(5);
  }
  for(int x = 0; x < 100; x++){ //move another 50 steps past the sensor
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
    }
  stepsPos = 0;
}

void left(int steps) {
  if (stepsPos+steps < maxRotation){//no problem, move as normal
    digitalWrite(dirPin,HIGH); // HIGH = left (counterclockwise), LOW = right (clockwise) with black towards power of the motor
    for(int x = 0; x < steps; x++){
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
    }
    stepsPos += steps;
  }else{//move back to 0 position before doing anything else
    Serial.println("Move back to 0 position before doing anything else");
    resetTo0();
  }
}

void right(int steps) {
  if (stepsPos-steps > 0){//no problem, move as normal
    digitalWrite(dirPin,LOW); // HIGH = left (counterclockwise), LOW = right (clockwise) with black towards power of the motor
    for(int x = 0; x < steps; x++){
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
    }
    stepsPos -= steps;
  }else{
    Serial.println("Move to max position before doing anything else");
    digitalWrite(dirPin,HIGH);
    for(int x = 0; x < maxRotation; x++){
      digitalWrite(stepPin,HIGH);
      delay(5);
      digitalWrite(stepPin,LOW);
      delay(5);
    }
    stepsPos = maxRotation;
  }
}
