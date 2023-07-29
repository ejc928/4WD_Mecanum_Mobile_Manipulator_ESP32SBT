#include <ESP32Servo.h>
#include <IRremote.h>
#include <DabbleESP32.h>
#include <NewPing.h> //used due to unreliability of manual computations
#include <Arduino.h>

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
const int PIN_RED   = 0;
const int PIN_WHITE = 15;

//*****MOTOR_DRIVER*****//

// Front Wheels
const int FL4 = 22;
const int FL3 = 21;
const int FR1 = 18;
const int FR2 = 19;

// Rear Wheels
const int BL1 = 23;
const int BL2 = 13;
const int BR4 = 27;
const int BR3 = 14;

//*****ROBOT_ARM*****//

//Declare object for 5 Servo Motors  
Servo Servo_0;
Servo Servo_1;
Servo Servo_3;
Servo Gripper;

//*****LOGIC,INTEGERS,BOOL*****//

//Last Code Received
unsigned long lastCode;

//initial angle  for servo
int angle = 0;
int angleStep = 2;

//Servo1
int baseA = 87;
//Servo2
int elbowB = 0;
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

  IrReceiver.begin(IRPIN, ENABLE_LED_FEEDBACK);

  //Declare the pins to which the Servo Motors are connected to 
  Servo_0.attach(2);
  Servo_1.attach(4);
  Servo_3.attach(16);
  Gripper.attach(17);

  //Write the servo motors to initial position 
  Servo_0.write(80);
  Servo_1.write(0);
  Servo_3.write(90);
  Gripper.write(0);

  pinMode(FL4, OUTPUT);
  pinMode(FL3, OUTPUT);
  pinMode(FR1, OUTPUT);
  pinMode(FR2, OUTPUT);
  pinMode(BL1, OUTPUT);
  pinMode(BL2, OUTPUT);
  pinMode(BR4, OUTPUT);
  pinMode(BR3, OUTPUT);

  pinMode(LineRight, INPUT_PULLUP);
  pinMode(LineLeft, INPUT_PULLUP);

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


//********************************MOTOR_FUNCTIONS*********************************************//
void motor() {
  int a = GamePad.getAngle();
  int b = GamePad.getRadius();
  
    //FORWARD
  if (a > 60 && a < 120) {
    goForward();
  }
    //BACKWARD
  else if (a > 240 && a < 300) {
    goBackward();
  }
    //LEFT
  else if (a > 150 && a < 210) {
    goLeft();
  }
    //RIGHT
  else if (a > 330 || a < 30 && b != 0) {
    goRight();
  }
    //FRONT-LEFT
  else if (a >= 120 && a <= 150) {
    diaUL();
  }
    //FRONT-RIGHT
  else if (a >= 30 && a <= 60) {
    diaUR();
  }
    //BACK-LEFT
  else if (a >= 210 && a <= 240) {
    diaLL();
  }
    //BACK-RIGHT
  else if (a >= 300 && a <= 330) {
    diaLR();
  }
    //spinRight
  else if (GamePad.isCirclePressed()) {
    spinRight();
  }
  else if (GamePad.isSquarePressed()) {
    spinLeft();
  }
  else{
    stop();
  }
}

void goForward() {
  Serial.println("Forward");
  digitalWrite(FL4, HIGH);
  digitalWrite(FL3, LOW);
  digitalWrite(FR2, HIGH);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, HIGH);
  digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void goBackward() {
  Serial.println("Backward");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, HIGH);
  digitalWrite(FR2, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, HIGH);
  digitalWrite(BR3, LOW);
  digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, LOW);
  digitalWrite(PIN_RED, HIGH);
}

void goLeft() {
  Serial.println("Left");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, HIGH);
  digitalWrite(FR2, HIGH);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void goRight() {
  Serial.println("Right");
  digitalWrite(FL4, HIGH);
  digitalWrite(FL3, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, HIGH);
  digitalWrite(BR3, HIGH);
  digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void spinLeft() {
  Serial.println("spinL");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, HIGH);
  digitalWrite(FR2, HIGH);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, HIGH);
  digitalWrite(BR3, HIGH);
  digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void spinRight() {
  Serial.println("spinR");
  digitalWrite(FL4, HIGH);
  digitalWrite(FL3, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void diaUL() {
  Serial.println("FL");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, HIGH);
  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
  digitalWrite(BR4, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void diaUR() {
  Serial.println("FR");
  digitalWrite(FL4, HIGH);
  digitalWrite(FL3, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, HIGH);
  digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void diaLL() {
Serial.println("BL");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, HIGH);
  digitalWrite(FR2, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(BR4, HIGH);
  digitalWrite(PIN_WHITE, LOW);
  digitalWrite(PIN_RED, HIGH);
}

void diaLR () {
  Serial.println("BR");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, HIGH);
  digitalWrite(BR4, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(PIN_WHITE, LOW);
  digitalWrite(PIN_RED, HIGH);
}


void stop() {
  Serial.println("Stop");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
  digitalWrite(BR4, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, HIGH);
}

void conL () {
  Serial.println("conL");
  digitalWrite(FL4, LOW);
  digitalWrite(FL3, LOW);
  digitalWrite(FR2, HIGH);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, HIGH);
  digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}

void conR () {
  Serial.println("conR");
  digitalWrite(FL4, HIGH);
  digitalWrite(FL3, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
  digitalWrite(BR3, LOW);
  digitalWrite(BR4, LOW);
  digitalWrite(PIN_WHITE, HIGH);
  digitalWrite(PIN_RED, LOW);
}


//******************************ROBOT_ARM_FUNCTIONS*******************************************//
void arm() { //Function to read the Analog value form POT and map it to Servo value
  int a = GamePad.getAngle();
  int b = GamePad.getRadius();
    //FORWARD
  if (a > 60 && a < 120 || GamePad.isUpPressed()) {
    shoulderC = shoulderC + angleStep;
    Servo_1.write(shoulderC);
    if(shoulderC>=90)
    {
      shoulderC = 90;
      Servo_1.write(shoulderC);
    }
    delay(20);
  }
    //BACKWARD
  if (a > 240 && a < 300 || GamePad.isDownPressed()) {
    shoulderC = shoulderC - angleStep;
    Servo_1.write(shoulderC);
    if(shoulderC<=0)
    {
      shoulderC = 0;
      Servo_1.write(shoulderC);
    }
    delay(20);
  }
    //LEFT
  if (a > 150 && a < 210 || GamePad.isLeftPressed()) {
    baseA = baseA + angleStep;
    Servo_0.write(baseA);
    if(baseA>=180)
    {
      baseA = 180;
      Servo_0.write(baseA);
    }
    delay(20);
  }
    //RIGHT
  if (a > 330 || a < 30 && b != 0 || GamePad.isRightPressed()) {
    baseA = baseA - angleStep;
    Servo_0.write(baseA);
    if(baseA<=0)
    {
      baseA = 0;
      Servo_0.write(baseA);
    }
    delay(20);
  }
  //UP DOWN
  if (GamePad.isTrianglePressed()) {
    elbowB = elbowB + angleStep;
    Servo_3.write(elbowB);
    if(elbowB>=90)
    {
      elbowB = 90;
      Servo_3.write(elbowB);
    }
    delay(20);
  }
  if (GamePad.isCrossPressed()) {
    elbowB = elbowB - angleStep;
    Servo_3.write(elbowB);
    if(elbowB<=0)
    {
      elbowB = 0;
      Servo_3.write(elbowB);
    }
    delay(20);
  }
  //Gripper
  if (GamePad.isCirclePressed()) {
    gripD = gripD + angleStep;
    Gripper.write(gripD);
    if(gripD>=90)
    {
      gripD = 90;
      Gripper.write(gripD);
    }
    delay(20);
  }
  if (GamePad.isSquarePressed()) {
    gripD = gripD - angleStep;
    Gripper.write(gripD);
    if(gripD<=0)
    {
      gripD = 0;
      Gripper.write(gripD);
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
      delay(100);
    } else if (LeftSensor > 15 && LeftSensor > 15 && FrontSensor > 15) {
      goForward();
    }
  } else if (FrontSensor <= 20) {
    if (RightSensor > 15 && LeftSensor <= 15) {
      spinRight();
      delay(100);
    } else if (LeftSensor > 15 && RightSensor <= 15) {
      spinLeft();
      delay(100);
    } else if (LeftSensor > 15 && RightSensor > 15) {
      spinRight();
      delay(100);
    } else if (LeftSensor <= 15 && RightSensor <= 15) {
      spinRight();
      delay(100);
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
      delay(100);
    } else if (LeftSensor > 15 && LeftSensor > 15 && FrontSensor > 15) {
      goForward();
    }
  } else if (FrontSensor <= 20) {
    if (RightSensor <= 15 && LeftSensor > 15) {
      spinLeft();
      delay(100);
    } else if (RightSensor > 15 && LeftSensor <= 15) {
      spinRight();
      delay(100);
    } else if (LeftSensor <= 15 && RightSensor <= 15) {
      spinLeft();
      delay(100);
    } else if (RightSensor > 15 && LeftSensor > 15) {
      spinLeft();
      delay(100);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void Reset() {
  Servo_0.write(80);
  Servo_1.write(0);
  Servo_3.write(90);
  Gripper.write(0);
  stop();
}

void Stop() {
  Servo_0.write(80);
  Servo_1.write(0);
  Servo_3.write(90);
  Gripper.write(0);
}
