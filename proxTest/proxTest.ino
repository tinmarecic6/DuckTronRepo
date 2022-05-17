#include <Servo.h>

int dir = 1;
const int dirPin = 2;
const int stepPin = 3;
const int stepsPerRevolution = 205;
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
  pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, HIGH);
  int i = 0;
  while (zeroPosTouched == false) {
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
    delay(5);
    proxVal = digitalRead(proximitySensorPin);
    Serial.println(proxVal);
    if (proxVal == 0) {
      zeroPosTouched = true;
      debug = "Touched";
    }
    Serial.println(debug);
    delay(10);
  }
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
