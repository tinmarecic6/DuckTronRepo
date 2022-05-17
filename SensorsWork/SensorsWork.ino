// Define pin connections & motor's steps per revolution 
const int proximitySensor = 4
const int dirPin = 2;
const int stepPin = 3;
const int stepsPerRevolution = 500;
const int pRTopRight = A0; // Photoresistor at Arduino analog pin A0 BLUE - top right
const int pRTopLeft = A1; // Photoresistor at Arduino analog pin A1 GREEN - top left
const int pRBottomRight = A2; // Photoresistor at Arduino analog pin A0 GREY - bottom right
const int pRBottomLeft = A3; // Photoresistor at Arduino analog pin A1 PURPLE - bottom left
int stepPos = 0;
int dir = 1; // 1 = clockwise, -1 = counterclockwise
bool startPosTouched = false;


void setup()
{
  // Declare pins as Outputs
  pinMode(proximitySensor, INPUT); 
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pRTopRight, INPUT);
  pinMode(pRTopLeft, INPUT);
  pinMode(pRBottomRight, INPUT);
  pinMode(pRBottomLeft, INPUT);
  Serial.begin(9600);
  while(startPosTouched == false){
    
  }
}
void loop()
{
  int proxStatus = digitalRead(proximitySensor);
  // Spin motor slowly
  /*
  Photoresistor read
  */
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    int val = analogRead(pRTopRight);
    Serial.print("pRTopRight ");
    Serial.println(val);
    int val2 = analogRead(pRTopLeft);
    Serial.print("pRTopLeft ");
    Serial.println(val2);
    int val3 = analogRead(pRBottomRight);
    Serial.print("pRBottomRight ");
    Serial.println(val3);
    int val4 = analogRead(pRBottomLeft);
    Serial.print("pRBottomLeft ");
    Serial.println(val4);
    delay(1000);
    /*
    Stepper motor code
    */
    if (stepPos >= stepsPerRevolution){
      dir = -1;
      digitalWrite(dirPin, LOW); // Set motor direction counterclockwise
    }else if (stepPos < 0){
      dir = 1;
      digitalWrite(dirPin, HIGH); // Set motor direction clockwise
    }
    stepPos += dir;
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
    delay(5);
  }
}
