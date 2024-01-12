#include <Adafruit_PWMServoDriver.h>
#include <IRremote.h>
#include <DabbleESP32.h>
#include <NewPing.h> //used due to unreliability of manual computations
#include <Arduino.h>
#include <Wire.h>

//AF Servo min max freq
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  650 // this is the 'maximum' pulse length count (out of 4096)
#define FREQ 50 // 60hz frequency
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Ultrasonic
#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.

NewPing sonar1(33, 34, MAX_DISTANCE);
NewPing sonar2(25, 35, MAX_DISTANCE);
NewPing sonar3(26, 32, MAX_DISTANCE);

int FrontSensor;
int LeftSensor;
int RightSensor;

//Infrared Line
const int LineLeft = 36;
const int LineRight = 39;

//Infrared Receiver
const int IRPIN = 5;

//LED
const int PIN_RED   = 15;
const int PIN_WHITE = 0;

//74HC595 Shift Register
const int latchPin = 4;      // Latch pin of 74HC595 is connected to Digital pin 5
const int clockPin = 2;      // Clock pin of 74HC595 is connected to Digital pin 6
const int dataPin = 16;       // Data pin of 74HC595 is connected to Digital pin 4

//Servo Driver
uint8_t Servo1 = 7;
uint8_t Servo2 = 6;
uint8_t Servo3 = 8;
uint8_t Servo4 = 9;

//Motor PWM
const int enaF = 23;
const int enbF = 19;
const int enaR = 27;
const int enbR = 14;
int min_speed = 85;
int max_speed = 170;


//*****LOGIC,INTEGERS,BOOL*****//

//Last Code Received
unsigned long lastCode;

//initial angle  for servo
int angle = 0;
int angleStep = 2;

//Servo1
int baseA = 80;
//Servo2
int elbowB = 80;
//Servo3
int shoulderC = 0;
//Servo4
int gripD = 0;

//statement
bool Drive = false;
bool Robot = false;
bool LFlwr = false;
bool Left = false;
bool Right = false;
bool Records = false;
bool Stops = false;
bool Resets = false;
bool Plays = false;
bool spinR = false;
bool spinL = false;

void setup() {
  Serial.begin(115200); //Serial Monitor for Debugging
  Dabble.begin("NodeMCU ESP32S");       //set bluetooth name of your device

  //AF Servo
  pwm.begin();
  pwm.setPWMFreq(FREQ);

  //Remote
  IrReceiver.begin(IRPIN, ENABLE_LED_FEEDBACK);

  //Motor Speed Output
  pinMode(enaF, OUTPUT);
  pinMode(enbF, OUTPUT);
  pinMode(enaR, OUTPUT);
  pinMode(enbR, OUTPUT);

  // Set all the pins of 74HC595 as OUTPUT
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);

  //IR
  pinMode(LineRight, INPUT_PULLUP);
  pinMode(LineLeft, INPUT_PULLUP);

  //LED
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_WHITE, OUTPUT);
}

void loop() {
  Dabble.processInput();

    //IR Receiver
  if (IrReceiver.decode()) {
  Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
  {
    if (IrReceiver.decodedIRData.decodedRawData == 0) {
      //REPEAT LAST CODE
      Serial.println("Last Code initialized");
      IrReceiver.decodedIRData.decodedRawData = lastCode;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xBA45FF00) {
      Serial.println("Manual Driving");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = true;
      Robot = false;
      LFlwr = false;
      Left = false;
      Right = false;
      Records = false;
      Stops = false;
      Resets = false;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xB946FF00) {
      Serial.println("Manual Robot");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = false;
      Robot = true;
      LFlwr = false;
      Left = false;
      Right = false;
      Records = false;
      Stops = false;
      Resets = false;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xB847FF00) {
      Serial.println("Line Tracking Mode");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = false;
      Robot = false;
      LFlwr = true;
      Left = false;
      Right = false;
      Records = false;
      Stops = false;
      Resets = false;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xF708FF00) {
      Serial.println("Left Wall Following");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = false;
      Robot = false;
      LFlwr = false;
      Left = true;
      Right = false;
      Records = true;
      Stops = false;
      Resets = false;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xA55AFF00) {
      Serial.println("Right Wall Following");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = false;
      Robot = false;
      LFlwr = false;
      Left = false;
      Right = true;
      Records = true;
      Stops = false;
      Resets = false;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xE31CFF00) {
      Serial.println("RESET");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = false;
      Robot = false;
      LFlwr = false;
      Left = false;
      Right = false;
      Records = false;
      Stops = false;
      Resets = true;
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xAD52FF00) {
      Serial.println("STOP");
      lastCode = IrReceiver.decodedIRData.decodedRawData;
      Drive = false;
      Robot = false;
      LFlwr = false;
      Left = false;
      Right = false;
      Records = false;
      Stops = true;
      Resets = false;
    }
  }
  IrReceiver.resume();
  }

  if (Drive == true) { //If user has selected Stop Mode 
    motor();
  }

  if (Robot == true) {//If user has selected Stop Mode 
    arm();
  }

  if (LFlwr == true) { //If user has selected Record mode
    Line();
  }

  if (Right == true) {//If user has selected Play Mode 
    mazefollowright();
  }

  if (Left == true) {//If user has selected Play Mode 
    mazefollowleft();
  }

  if (Stops == true) {//If user has selected Stop Mode 
    Stop();
  }

  if (Resets == true) {//If user has selected Stop Mode 
    Reset();
  }
}

//*****************************SHIFT_REGISTER_UPDATE******************************************//
void updateShiftRegister( const uint8_t value )
{
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, value ); 
    digitalWrite(latchPin, HIGH);
}


//********************************MOTOR_FUNCTIONS*********************************************//
void motor() {
  int a = GamePad.getAngle();
  int b = GamePad.getRadius();
  int motor_speed;
  
  if (b >= 6) {
    b = 6;
  }

  motor_speed = map(b, 1, 6, min_speed, max_speed);

    //FORWARD
  if (a > 60 && a < 120) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    goForward();
  }
    //BACKWARD
  else if (a > 240 && a < 300) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    goBackward();
  }
    //LEFT
  else if (a > 150 && a < 210) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    goLeft();
  }
    //RIGHT
  else if (a > 330 || a < 30 && b != 0) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    goRight();
  }
    //FRONT-LEFT
  else if (a >= 120 && a <= 150) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    diaUL();
  }
    //FRONT-RIGHT
  else if (a >= 30 && a <= 60) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    diaUR();
  }
    //BACK-LEFT
  else if (a >= 210 && a <= 240) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    diaLL();
  }
    //BACK-RIGHT
  else if (a >= 300 && a <= 330) {
    analogWrite(enaF, motor_speed);
    analogWrite(enbF, motor_speed);
    analogWrite(enaR, motor_speed);
    analogWrite(enbR, motor_speed);
    diaLR();
  }
    //spinRight
  else if (GamePad.isCirclePressed()) {
    analogWrite(enaF, max_speed);
    analogWrite(enbF, max_speed);
    analogWrite(enaR, max_speed);
    analogWrite(enbR, max_speed);
    spinRight();
  }
    //spinLeft
  else if (GamePad.isSquarePressed()) {
    analogWrite(enaF, max_speed);
    analogWrite(enbF, max_speed);
    analogWrite(enaR, max_speed);
    analogWrite(enbR, max_speed);
    spinLeft();
  }
  else{
    stop();
  }
}

void goForward() {
  Serial.println("Forward");
  updateShiftRegister( 0b10101010 );
  //digitalWrite(FL4, HIGH);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR2, HIGH);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, HIGH);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, HIGH);
  //digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void goBackward() {
  Serial.println("Backward");
  updateShiftRegister( 0b01010101 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, HIGH);
  //digitalWrite(FR2, LOW);
  //digitalWrite(FR1, HIGH);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, HIGH);
  //digitalWrite(BR3, LOW);
  //digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, LOW);
  digitalWrite(PIN_RED, HIGH);
}

void goLeft() {
  Serial.println("Left");
  updateShiftRegister( 0b10010110 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, HIGH);
  //digitalWrite(FR2, HIGH);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, HIGH);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, LOW);
  //digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void goRight() {
  Serial.println("Right");
  updateShiftRegister( 0b01101001 );
  //digitalWrite(FL4, HIGH);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR2, LOW);
  //digitalWrite(FR1, HIGH);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, HIGH);
  //digitalWrite(BR3, HIGH);
  //digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void spinLeft() {
  Serial.println("spinL");
  updateShiftRegister( 0b01100110 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, HIGH);
  //digitalWrite(FR2, HIGH);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, HIGH);
  //digitalWrite(BR3, HIGH);
  //digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void spinRight() {
  Serial.println("spinR");
  updateShiftRegister( 0b10011001 );
  //digitalWrite(FL4, HIGH);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR2, LOW);
  //digitalWrite(FR1, HIGH);
  //digitalWrite(BL1, HIGH);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, LOW);
  //digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void diaUL() {
  Serial.println("FL");
  updateShiftRegister( 0b10000010 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR1, LOW);
  //digitalWrite(FR2, HIGH);
  //digitalWrite(BL1, HIGH);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR4, LOW);
  //digitalWrite(BR3, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void diaUR() {
  Serial.println("FR");
  updateShiftRegister( 0b00101000 );
  //digitalWrite(FL4, HIGH);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR2, LOW);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, HIGH);
  //digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void diaLL() {
Serial.println("BL");
updateShiftRegister( 0b00010100 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, HIGH);
  //digitalWrite(FR2, LOW);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, LOW);
  //digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, LOW);
  digitalWrite(PIN_RED, HIGH);
}

void diaLR () {
  Serial.println("BR");
  updateShiftRegister( 0b01000001 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR1, HIGH);
  //digitalWrite(FR2, LOW);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, HIGH);
  //digitalWrite(BR4, LOW);
  //digitalWrite(BR3, LOW);
  digitalWrite(PIN_WHITE, LOW);
  digitalWrite(PIN_RED, HIGH);
}


void stop() {
  Serial.println("Stop");
  updateShiftRegister( 0 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR1, LOW);
  //digitalWrite(FR2, LOW);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR4, LOW);
  //digitalWrite(BR3, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void conL () {
  Serial.println("conL");
  updateShiftRegister( 0b01000100 );
  //digitalWrite(FL4, LOW);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR2, HIGH);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, LOW);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, HIGH);
  //digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void conR () {
  Serial.println("conR");
  updateShiftRegister( 0b00010001 );
  //digitalWrite(FL4, HIGH);
  //digitalWrite(FL3, LOW);
  //digitalWrite(FR2, LOW);
  //digitalWrite(FR1, LOW);
  //digitalWrite(BL1, HIGH);
  //digitalWrite(BL2, LOW);
  //digitalWrite(BR3, LOW);
  //digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}


//******************************ROBOT_ARM_FUNCTIONS*******************************************//
int angletoPulse(int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void arm() { //Function to read the Analog value form POT and map it to Servo value
  int a = GamePad.getAngle();
  int b = GamePad.getRadius();
    //FORWARD
  if (a > 60 && a < 120 || GamePad.isUpPressed()) {
    shoulderC = shoulderC + angleStep;
    pwm.setPWM(Servo1, 0, angletoPulse(shoulderC) );
    if(shoulderC>=90)
    {
      shoulderC = 90;
      pwm.setPWM(Servo1, 0, angletoPulse(shoulderC) );
    }
    delay(20);
  }
    //BACKWARD
  if (a > 240 && a < 300 || GamePad.isDownPressed()) {
    shoulderC = shoulderC - angleStep;
    pwm.setPWM(Servo1, 0, angletoPulse(shoulderC) );
    if(shoulderC<=0)
    {
      shoulderC = 0;
      pwm.setPWM(Servo1, 0, angletoPulse(shoulderC) );
    }
    delay(20);
  }
    //LEFT
  if (a > 150 && a < 210 || GamePad.isLeftPressed()) {
    baseA = baseA + angleStep;
    pwm.setPWM(Servo2, 0, angletoPulse(baseA) );
    if(baseA>=180)
    {
      baseA = 180;
      pwm.setPWM(Servo2, 0, angletoPulse(baseA) );
    }
    delay(20);
  }
    //RIGHT
  if (a > 330 || a < 30 && b != 0 || GamePad.isRightPressed()) {
    baseA = baseA - angleStep;
    pwm.setPWM(Servo2, 0, angletoPulse(baseA) );
    if(baseA<=0)
    {
      baseA = 0;
      pwm.setPWM(Servo2, 0, angletoPulse(baseA) );
    }
    delay(20);
  }
  //UP DOWN
  if (GamePad.isTrianglePressed()) {
    elbowB = elbowB + angleStep;
    pwm.setPWM(Servo3, 0, angletoPulse(elbowB) );
    if(elbowB>=90)
    {
      elbowB = 90;
      pwm.setPWM(Servo3, 0, angletoPulse(elbowB) );
    }
    delay(20);
  }
  if (GamePad.isCrossPressed()) {
    elbowB = elbowB - angleStep;
    pwm.setPWM(Servo3, 0, angletoPulse(elbowB) );
    if(elbowB<=0)
    {
      elbowB = 0;
      pwm.setPWM(Servo3, 0, angletoPulse(elbowB) );
    }
    delay(20);
  }
  //Gripper
  if (GamePad.isCirclePressed()) {
    gripD = gripD + angleStep;
    pwm.setPWM(Servo4, 0, angletoPulse(gripD) );
    if(gripD>=60)
    {
      gripD = 60;
      pwm.setPWM(Servo4, 0, angletoPulse(gripD) );
    }
    delay(20);
  }
  if (GamePad.isSquarePressed()) {
    gripD = gripD - angleStep;
    pwm.setPWM(Servo4, 0, angletoPulse(gripD) );
    if(gripD<=0)
    {
      gripD = 0;
      pwm.setPWM(Servo4, 0, angletoPulse(gripD) );
    }
    delay(20);
  }
}


//**************************************Line_Follower**************************************//
void Line() {
  if (digitalRead(LineLeft) == 0 && digitalRead(LineRight) == 1) {
    conR();
  }
  else if (digitalRead(LineRight) == 0 && digitalRead(LineLeft) == 1) {
    conL();
  }
  else if (digitalRead(LineLeft) == 1 && digitalRead(LineRight) == 1) {
    stop();
  } else {
    goForward();
  }
}


//**************************************Wall_Follower**************************************//
void distance() {
  FrontSensor = sonar1.ping_cm();
  Serial.print("F: ");
  Serial.print(sonar1.ping_cm());
  Serial.print("\t");
  LeftSensor = sonar2.ping_cm();
  Serial.print("L: ");
  Serial.print(sonar2.ping_cm());
  Serial.print("\t");
  RightSensor = sonar3.ping_cm();
  Serial.print("R: ");
  Serial.print(sonar3.ping_cm());
  Serial.print("\t");
  Serial.println("");
  delay(50);
}

void mazefollowright() {

  distance();

  if (FrontSensor > 20) //Free space forward
  {
    if (RightSensor >= 7 && RightSensor <= 15) {
      goForward();
    } else if (RightSensor < 7) {
      diaUL();
    } else if (RightSensor > 15) {
      diaUR();
    } else if (RightSensor > 15 && LeftSensor <= 15) {
      spinRight();
    } else if (LeftSensor > 15 && LeftSensor > 15 && FrontSensor > 15) {
      goForward();
    }
  } else if (FrontSensor <= 20) {
    if (RightSensor > 15 && LeftSensor <= 15) {
      spinRight();
    } else if (LeftSensor > 15 && RightSensor <= 15) {
      spinLeft();
    } else if (LeftSensor > 15 && RightSensor > 15) {
      spinRight();
    } else if (LeftSensor <= 15 && RightSensor <= 15) {
      spinRight();
    }
  }
}

void mazefollowleft() {
  
  distance();

  if (FrontSensor > 20) //Free space forward
  {
    if (LeftSensor >= 7 && LeftSensor <= 15) {
      goForward();
    } else if (LeftSensor < 7) {
      diaUR();
    } else if (LeftSensor > 15) {
      diaUL();
    } else if (RightSensor <= 15 && LeftSensor > 15) {
      spinLeft();
    } else if (LeftSensor > 15 && LeftSensor > 15 && FrontSensor > 15) {
      goForward();
    }
  } else if (FrontSensor <= 20) {
    if (RightSensor <= 15 && LeftSensor > 15) {
      spinLeft();
    } else if (RightSensor > 15 && LeftSensor <= 15) {
      spinRight();
    } else if (LeftSensor <= 15 && RightSensor <= 15) {
      spinLeft();
    } else if (RightSensor > 15 && LeftSensor > 15) {
      spinLeft();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void Reset() {
  pwm.setPWM(Servo2, 0, angletoPulse(80));
  pwm.setPWM(Servo3, 0, angletoPulse(80));
  pwm.setPWM(Servo1, 0, angletoPulse(0));
  pwm.setPWM(Servo4, 0, angletoPulse(0));
}

void Stop() {
 //
}
