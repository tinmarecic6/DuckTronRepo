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
//From the front photoresistors
const int pRTopRightPin = A1; // Photoresistor at Arduino analog pin A0 BLUE - top right
const int pRTopLeftPin = A0; // Photoresistor at Arduino analog pin A1 GREEN - top left
const int pRBottomRightPin = A3; // Photoresistor at Arduino analog pin A0 GREY - bottom right
const int pRBottomLeftPin = A2; // Photoresistor at Arduino analog pin A1 PURPLE - bottom left
int topRightResVal;
int topLeftResVal;
int bottomRightResVal;
int bottomLeftResVal;
int photoOffset = 5;
Servo myservo;
int servoStepSize = 1;
int servoMaxPos = 45;
int servoCurPos = 0;



void setup() {
  myservo.attach(4);
  myservo.write(0);
  Serial.begin(9600);
  pinMode(proximitySensorPin, INPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(proximitySensorPin);
  Serial.println(val);
  topRightResVal = 0;
  topLeftResVal = 0;
  bottomRightResVal = 0;
  bottomLeftResVal = 0;
  int divider = 10;
  for (int i = 0; i < divider; i++) {
    topRightResVal += analogRead(pRTopRightPin);
    topLeftResVal += analogRead(pRTopLeftPin);
    bottomRightResVal += analogRead(pRBottomRightPin);
    bottomLeftResVal += analogRead(pRBottomLeftPin);
    delay(100);
  }
  topRightResVal = topRightResVal/divider;
  topLeftResVal = topLeftResVal/divider;
  bottomRightResVal = bottomRightResVal/divider;
  bottomLeftResVal = bottomLeftResVal/divider;
  Serial.println("\n----------");
  Serial.print("Top right:");
  Serial.println(topRightResVal);
  Serial.print("Top left:");
  Serial.println(topLeftResVal);
  Serial.print("Bottom right:");
  Serial.println(bottomRightResVal);
  Serial.print("Bottom left:");
  Serial.println(bottomLeftResVal);
  int bottomVal = bottomLeftResVal + bottomRightResVal;
  int topVal = topLeftResVal + topRightResVal;
  int rightVal = topRightResVal + bottomRightResVal;
  int leftVal = topLeftResVal + bottomLeftResVal;

  if (bottomVal + (photoOffset * 3) < topVal) {
    //move up
    Serial.println("Moving up");
    //up();
  }
  else if (topVal + (photoOffset * 3) < bottomVal && (servoCurPos - servoStepSize < 0)) {
    //move down
    Serial.println("Moving down");
    //down();

  }
  else if (leftVal + (photoOffset * 3) < rightVal) {
    //move right
    Serial.println("Moving  right");
    //right(5);
  }
  else if (rightVal + (photoOffset * 3) < leftVal) {
    //move left
    Serial.println("Moving left");
    ///left(5);
  }
  else {
    if (topLeftResVal > (topRightResVal + photoOffset) && bottomLeftResVal > (bottomRightResVal + photoOffset)) {
      //move to the left
      Serial.println("Moving left");
      //left(5);
    } else if (topRightResVal > (topLeftResVal + photoOffset) && bottomRightResVal > (bottomLeftResVal + photoOffset)) {
      Serial.println("Moving right");
      //move to the right
      //right(5);
    }
    if (topLeftResVal > (bottomLeftResVal + photoOffset) && topRightResVal > (bottomRightResVal + photoOffset)) {
      //move up
      Serial.println("Moving up");
      //moveUp();
    } else if (bottomLeftResVal > (topLeftResVal + photoOffset) && bottomRightResVal > (topRightResVal + photoOffset)) {
      //move down
      Serial.println("Moving down");
      //moveDown();
    }
  }
  delay(1000);
}

void resetTo0() {
  Serial.println("Resetting to 0 position");
  digitalWrite(dirPin, LOW);
  while (digitalRead(proximitySensorPin) != 0) {
    Serial.println("Not touched");
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
    delay(5);
  }
  Serial.println("Moving another 100 steps");
  delay(1000);
  for (int x = 0; x < 100; x++) { //move another 100 steps past the sensor
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
    delay(5);
  }
  stepsPos = 0;
}

void left(int steps) {
  Serial.println("Moving to the left");
  if (stepsPos + steps < maxRotation) { //no problem, move as normal
    digitalWrite(dirPin, HIGH); // HIGH = left (counterclockwise), LOW = right (clockwise) with black towards power of the motor
    for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delay(5);
      digitalWrite(stepPin, LOW);
      delay(5);
    }
    stepsPos += steps;
  } else { //move back to 0 position before doing anything else
    Serial.println("Move back to 0 position before doing anything else");
    resetTo0();
  }
}

void right(int steps) {
  Serial.println("Moving to the right");
  if (stepsPos - steps > 0) { //no problem, move as normal
    digitalWrite(dirPin, LOW); // HIGH = left (counterclockwise), LOW = right (clockwise) with black towards power of the motor
    for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delay(5);
      digitalWrite(stepPin, LOW);
      delay(5);
    }
    stepsPos -= steps;
  } else {
    Serial.println("Move to max position before doing anything else");
    digitalWrite(dirPin, HIGH);
    for (int x = 0; x < maxRotation; x++) {
      digitalWrite(stepPin, HIGH);
      delay(5);
      digitalWrite(stepPin, LOW);
      delay(5);
    }
    stepsPos = maxRotation;
  }
}

void up() {
  if (servoCurPos + servoStepSize >= servoMaxPos) {
    Serial.println("Currently at max pos, not moving up, turning around instead");
    delay(2000);
    if (stepsPos + (maxRotation / 2) <= maxRotation) {
      left(maxRotation / 2);
    } else if (stepsPos - (maxRotation / 2) >= 0) {
      right(maxRotation / 2);
    } else {
      Serial.println("Should never happen");
    }
  }
  else {
    Serial.println("Moving up");
    servoCurPos += servoStepSize ;
    myservo.write(servoCurPos);
  }
}
void down() {
  if (servoCurPos - servoStepSize < 0) {
    Serial.println("Currently at 0 pos, not moving down");
  }
  else {
    Serial.println("Moving down");
    servoCurPos -= servoStepSize ;
    myservo.write(servoCurPos);
  }
}
