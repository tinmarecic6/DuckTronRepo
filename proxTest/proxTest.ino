#include <Servo.h>

int dir = 1; // 1 = right, -1 = left with black towards power of the motor
const int dirPin = 2;
const int stepPin = 3;
const int stepsPerRevolution = 200;
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
int maxRotation;
int photoOffset = 3;
Servo myservo;



void setup() {
  Serial.begin(9600);
  pinMode(proximitySensorPin, INPUT); 
  pinMode(dirPin, OUTPUT); 
  pinMode(stepPin,OUTPUT);
  digitalWrite(dirPin,HIGH);
  // Test if the motor is already above the sensor, if so move it away and back to get the correct 0 position
  if (digitalRead(proximitySensorPin) == 0){
    debug = "Touched before moving, correting position";
    digitalWrite(dirPin,LOW); //set direction and move it until it no longer touches the sensor
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
  while(zeroPosTouched == false){
    digitalWrite(dirPin,HIGH);
    proxVal = digitalRead(proximitySensorPin);
    if (proxVal == 0){
      zeroPosTouched = true;
      debug = "Touched";
    }
    digitalWrite(stepPin,HIGH);
    delay(5);
    digitalWrite(stepPin,LOW);
    delay(5);
    Serial.println(debug);
    delay(10);
  }
  Serial.println("Calibration done, waiting 5 seconds before starting");
  delay(5000);
  digitalWrite(dirPin, LOW);
  for (int x = 0; x < stepsPerRevolution; x++) {
    //stepPos += 1;
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
    delay(5);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(proximitySensorPin);

  topRightResVal = analogRead(pRTopRightPin);
  topLeftResVal = analogRead(pRTopLeftPin);
  bottomRightResVal = analogRead(pRBottomRightPin);
  bottomLeftResVal = analogRead(pRBottomLeftPin);
  Serial.println("\n----------");
  Serial.print("Top right:");
  Serial.println(topRightResVal);
  Serial.print("Top left:");
  Serial.println(topLeftResVal);
  Serial.print("Bottom right:");
  Serial.println(bottomRightResVal);
  Serial.print("Bottom left:");
  Serial.println(bottomLeftResVal);
  if (topLeftResVal > (topRightResVal + photoOffset) || bottomLeftResVal > (bottomRightResVal + photoOffset)) {
    //move to the left
    Serial.println("Moving to the left");
  }
  if (topLeftResVal > (topRightResVal + photoOffset) || bottomLeftResVal > (bottomRightResVal + photoOffset)) {
    //move to the right
    Serial.println("Moving to the right");
  }
  if (topLeftResVal > (bottomLeftResVal + photoOffset) || topRightResVal > (topRightResVal + photoOffset)) {
    //move up
    Serial.println("Moving up");
  }
  if (bottomLeftResVal > (topLeftResVal + photoOffset) || bottomRightResVal > (topRightResVal + photoOffset)) {
    //move down
    Serial.println("Moving down");
  }


  delay(5000);
}

void left() {
  if (stepsPos > maxRotation){
    
  }
  stepsPos += 1; // not sure if this is correct
}
