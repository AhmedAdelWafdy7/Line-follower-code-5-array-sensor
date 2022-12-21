/**
@ Ahmed Adel Wafdy 
@ Line follower 5 channel sensor
@ Ayman the robot
*/
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);
float pTerm, iTerm, dTerm;
int error;
int previousError;
float kp = 11; //11
float ki = 0;
float kd = 11; //11
float output;
int integral, derivative;
int irSensors[] = {A0, A1, A2, A3, 9}; //IR sensor pins
int irReadings[5];
int motor1Forward = 2;
int motor1Backward = 3;
/**
@ahmed adel wafdy
@line follower 5 channel sensor
*/
int motor1pwmPin = 5; 
int motor2Forward = 8;
int motor2Backward = 7;
int motor2pwmPin = 6;
int motor1newSpeed;
int motor2newSpeed;
int motor2Speed = 70; //Default 70
int motor1Speed = 120; //Default 120
int counter=0;
int irSensor = B00000;
void setup() {
  //Declare all IR sensors as inputs
  for (int pin = 0; pin < 5; pin++) {
    int pinNum = irSensors[pin];
    pinMode(pinNum, INPUT);
  }
  pinMode(motor1Forward, OUTPUT);
  pinMode(motor1Backward, OUTPUT);
  pinMode(motor1pwmPin, OUTPUT);
  pinMode(motor2Forward, OUTPUT);
  pinMode(motor2Backward, OUTPUT);
  pinMode(motor2pwmPin, OUTPUT);
  lcd.init();                     
 lcd.init();
 lcd.backlight();
    
 
  
 lcd.setCursor(3,0);
 lcd.print("Ayman     ");
}

void loop() {
  //Put all of our functions here
  readIRSensors();
  calculateError();
  pidCalculations();
  changeMotorSpeed();
  if(counter==0){
  lcd.setCursor(3,0);
 lcd.print("Ayman  S     ");
   }
   else if(counter==1||counter==2||counter==3){
   lcd.setCursor(3,0);
 lcd.print("Ayman  A     ");
   }
  else if(counter==4||counter==5||counter==6){
   lcd.setCursor(3,0);
 lcd.print("Ayman  B     ");
   }
   else if(counter==8||counter==9||counter==10){
   lcd.setCursor(3,0);
 lcd.print("Ayman  C     ");
   }
   else if(counter>=11){
    lcd.setCursor(3,0);
 lcd.print("Ayman  D     ");
   }
   
   lcd.setCursor(0,0);
 lcd.print(counter);
 
  
   
  LcdLogic(irSensor);
}

void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  int ret = B00000;
  for (int pin = 0; pin < 5; pin++) {
    int pinNum = irSensors[pin];
    irReadings[pin] = digitalRead(pinNum);
  }
   for (int i = 0; i < 5; i++) {
    if (irReadings[i] == HIGH) ret += (0x1 << i);
  }
  irSensor=ret;    
}

void calculateError() {
  //Determine an error based on the readings
  if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 0)) {
    error = 4;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = 3;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 0) && (irReadings[4] == 1)) {
    error = 2;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 1)) {
    error = 1;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 0) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = 0;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = -1;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 0) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = -2;
  } else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = -3;
  } else if ((irReadings[0] == 0) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = -4;
  } else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) {    
    error = 0;
  } else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
  digitalWrite(motor2Forward, LOW);
  digitalWrite(motor2Backward, LOW);
  digitalWrite(motor1Forward, LOW);
  digitalWrite(motor1Backward, LOW);
  }
}

void pidCalculations() {
  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output = pTerm + iTerm + dTerm;
  previousError = error;
}

void changeMotorSpeed() {
  //Change motor speed of both motors accordingly
  motor2newSpeed = motor2Speed + output;
  motor1newSpeed = motor1Speed - output;
  //Constrain the new speed of motors to be between the range 0-255
  constrain(motor2newSpeed, 0, 255);
  constrain(motor1newSpeed, 0, 255);
  //Set new speed, and run motors in forward direction
  analogWrite(motor2pwmPin, motor2newSpeed);
  analogWrite(motor1pwmPin, motor1newSpeed);
  digitalWrite(motor2Forward, HIGH);
  digitalWrite(motor2Backward, LOW);
  digitalWrite(motor1Forward, HIGH);
  digitalWrite(motor1Backward, LOW);
}


void LcdLogic(int X) {
  switch (X) {
    case B00000:
     lcd.setCursor(1,1);
     lcd.print(" LAST ACTION      ");
      break;
    case B11111:
    case B01110:
  //case B01111:
  //case B11110:
     lcd.setCursor(1,1);
     lcd.print("      GO           ");
    counter++;
    delay(62);
      break;
    case B00010:
    case B00110:
      lcd.setCursor(1,1);
      lcd.print("    RIGHT          ");
      break;
    case B00001:
    case B00011:
    //case B00111:
      lcd.setCursor(1,1);
      lcd.print("  MORE RIGHT          ");
      break;
    case B00100:
      lcd.setCursor(1,1);
      lcd.print("      GO            ");
      break;
    case B01000:
    case B01100:
      lcd.setCursor(1,1);
      lcd.print("    LEFT          ");
      break;
    case B10000:
    case B11000:
    //case B11100:
      lcd.setCursor(1,1);
      lcd.print("  MORE LEFT          ");
      break;
    default:
      
      break;
  }
}