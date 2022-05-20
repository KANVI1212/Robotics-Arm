/*
  Project: Bracc.ino
  Date: 09/01/2020
  Last change: 23/01/2020

  Team: Basso Andrea
        Delmoro Niccol
        Gramaglia Giacomo
  //////JOYPAD CODE//////
*/
////////////////////
////DECLARATIONS////
////////////////////

/*==LIBRARIES==*/
#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // HC-05 = 2-RX(+resistors) | 3-TX
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd (0x27, 16, 2); //sda=A4 , scl=A5

/*==BUTTONS==*/
int button1Pin = 4;       //Open Gripper
int valButton1;
int button2Pin = 5;       //Save Point
int valButton2;
int button3Pin = 7;       //Close Gripper
int valButton3;
int button4Pin = 8;       //Move sequence
int valButton4;
//Joystick Buttons
int button5Pin = 9;       //Coordinate Mode (NC)
int valButton5;
int button6Pin = 6;       //Home position
int valButton6;

/*==JOYSTICK 1==*/
int joy1xPin = A0;
int joy1yPin = A1;
int valX1;
int valY1;

/*==JOYSTICK 2==*/
int joy2xPin = A2;
int joy2yPin = A3;
int valX2;
int valY2;


/*==ANGLE VARIABLES==*/
//Base
int base = 90;
int baseMax = 180;
int baseMin = 0;
//Servo1
int servo1 = 73;
int servo1Max = 180;
int servo1Min = 0;
//Servo2
int servo2 = 26;
int servo2Max = 180;
int servo2Min = 0;
//Servo3
int servo3 = 147;
int servo3Max = 180;
int servo3Min = 0;
//Servo4
int servo4 = 14;
int servo4Max = 180;
int servo4Min = 0;

/*==Movement Commands==*/
char MDM = '0';       //Turn on direct motor mode
char MB1 = 'A';       //Move base Clockwise
char MB2 = 'B';       //Move base Counterclockwise
char M11 = 'C';       //Move servo1 Clockwise
char M12 = 'D';       //Move servo1 Counterclockwise
char M21 = 'E';       //Move servo2 Clockwise
char M22 = 'F';       //Move servo2 Counterclockwise
char M31 = 'G';       //Move servo3 Clockwise
char M32 = 'H';       //Move servo3 Counterclockwise
char M41 = 'I';       //Move servo4 Clockwise
char M42 = 'J';       //Move servo4 Counterclockwise
char MG1 = 'K';       //Open gripper
char MG2 = 'L';       //Close gripper
char MHO = 'M';       //Move home point
char MPS = 'N';       //Move points sequence
/*==Point Commands==*/
char PRD = 'P';       //Reading point
/*==Coordinate Commands==*/
char CMM = '1';       //Turn in coordinate Mode
char CX1 = 'R';       //Increase X value
char CX2 = 'S';       //Decrease X value
char CY1 = 'T';       //Increase Y value
char CY2 = 'U';       //Decrease Y value
char CZ1 = 'V';       //Increase Z angle
char CZ2 = 'W';       //Decrease Z angle
char CA1 = 'X';       //Grip angle increase-clockwise
char CA2 = 'Y';       //Grip angle decrease-counterclockwise

/*==TIME VALUE==*/
int Speed = 8;

/*==IK VALUES==*/
int XYMode;
int modeTouch;
int x = 95;
int y = 80;
int angle = 90;
int DB;

////////////////////
////////SETUP///////
////////////////////

void setup()
{
  //Button declarations
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);
  pinMode(button4Pin, INPUT);
  pinMode(button5Pin, INPUT);
  pinMode(button6Pin, INPUT);

  //Joysticks declarations
  pinMode(joy1xPin, INPUT);
  pinMode(joy1yPin, INPUT);
  pinMode(joy2xPin, INPUT);
  pinMode(joy2yPin, INPUT);

  //Begins
  Serial.begin(9600);
  BTserial.begin(38400);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Inizialitation...");
}

////////////////////
////////LOOP////////
////////////////////

void loop()
{
  valButton5 = digitalRead(button5Pin);

  //Mode Selection
  if (valButton5 == LOW && modeTouch == LOW)   //Enter in IK Mode
  {
    XYMode = 1;
    modeTouch = 1;
    delay(Speed);
    lcd.clear();
    BTserial.write(CMM);
    lcd.setCursor(0, 0);
    lcd.print("Coordinate XY");
    lcd.setCursor(0, 1);
    lcd.print("Mode...");
    //Initialitation
    x = 95;
    y = 80;
    angle = 90;
    base = 90;
    delay(1000);
  }
  else if (valButton5 == LOW && modeTouch == HIGH)  //Exit from IK Mode
  {
    XYMode = 0;
    modeTouch = 0;
    delay(Speed);
    lcd.clear();
    BTserial.write(MDM);
    lcd.setCursor(0, 0);
    lcd.print("Direct motor");
    lcd.setCursor(0, 1);
    lcd.print("Mode...");
    //Inizialitation
    base = 90;
    servo1 = 73;
    servo2 = 26;
    servo3 = 147;
    servo4 = 14;
    delay(1000);
  }

  //Mode actions [IK - DIRECT]
  if (XYMode == 1)           //IK Mode
  {
    coordinateMode();
  }
  else if (XYMode == 0)      //Direct Mode
  {
    readMovement();
  }
  reading();
}

////////////////////
//////READING///////
////////////////////

void reading()
{
  valButton1 = digitalRead(button1Pin);
  valButton2 = digitalRead(button2Pin);
  valButton3 = digitalRead(button3Pin);
  valButton4 = digitalRead(button4Pin);
  valButton6 = digitalRead(button6Pin);

  /*===== MG1 - MG2 = GRIPPER =====*/
  if (valButton1 == HIGH)             //LowButtonLeft - Gripper opening
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(MG1);
    lcd.setCursor(0, 0);
    lcd.print("Gripper =");
    lcd.setCursor(0, 1);
    lcd.print("Opened");
  }
  else if (valButton3 == HIGH)      //LowButtonRight- Gripper Closing
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(MG2);
    lcd.setCursor(0, 0);
    lcd.print("Gripper =");
    lcd.setCursor(0, 1);
    lcd.print("Closed");
  }

  /*===== Homing Button =====*/
  if (valButton6 == HIGH)            //JoystickRightButton - Active Home
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(MHO);
    lcd.setCursor(0, 0);
    lcd.print("Homing");
    base = 90;
    servo1 = 73;
    servo2 = 26;
    servo3 = 147;
    servo4 = 14;

    x = 95;
    y = 80;
    angle = 90;
    delay(1500);
  }

  /*===== Points =====*/
  if (valButton2 == HIGH)           //HighButtonLeft - Save point position
  {
    delay(Speed);
    lcd.clear();
    lcd.setCursor(0, 0);
    BTserial.write(PRD);
    savePoint();
    delay(1500);
  }
  if (valButton4 == HIGH)           //HighButtonRight - Move points sequence
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(MPS);
    lcd.setCursor(0, 0);
    lcd.print("Move points");
    lcd.setCursor(0, 1);
    lcd.print("sequence");
    delay(1500);
  }
}

void limits()  //Limit angle between angleMax and angle Min
{
  //base limitation
  if (base > baseMax)
    base = baseMax;
  else if (base < baseMin)
    base = baseMin;
  //Servo1 limitation
  if (servo1 > servo1Max)
    servo1 = servo1Max;
  else if (servo1 < servo1Min)
    servo1 = servo1Min;
  //Servo2 limitation
  if (servo2 > servo2Max)
    servo2 = servo2Max;
  else if (servo2 < servo2Min)
    servo2 = servo2Min;
  //Servo3 limitation
  if (servo3 > servo3Max)
    servo3 = servo3Max;
  else if (servo3 < servo3Min)
    servo3 = servo3Min;
  //Servo4 limitation
  if (servo4 > servo4Max)
    servo4 = servo4Max;
  else if (servo4 < servo4Min)
    servo4 = servo4Min;
}

////////////////////
//////DIRECT////////
////////////////////

void readMovement()
{
  valX1 = analogRead(joy1xPin);
  valY1 = analogRead(joy1yPin);
  valX2 = analogRead(joy2xPin);
  valY2 = analogRead(joy2yPin);

  /*===== MB1 - MB2 = BASE =====*/
  if (valX1 > 700 && valX2 > 700)             //joy1-2 SenseLeft - Base Clockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(MB1);
    base = base + 1;
    lcd.setCursor(0, 0);
    lcd.print("Base Angle=");
    lcd.setCursor(0, 1);
    lcd.print(base);
  }
  else if (valX1 < 350 && valX2 < 350)       //joy1-2 SenseRight - Base Counterclockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(MB2);
    base = base - 1;
    lcd.setCursor(0, 0);
    lcd.print("Base Angle=");
    lcd.setCursor(0, 1);
    lcd.print(base);
  }

  /*===== M11 - M12 = SERVO 1 =====*/
  if (valY1 > 700 && valY2 > 700)            //Joy1-2 SenseDown - Servo1 Clockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M11);
    servo1 = servo1 + 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo1 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo1);
  }
  else if (valY1 < 350 && valY2 < 350)       //Joy1-2 SenseUp - Servo1 Counterclockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M12);
    servo1 = servo1 - 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo1 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo1);
  }

  /*===== M21 - M22 = SERVO 2 =====*/
  if (valY1 < 350 && valY2 > 400 && valY2 < 600)           //Joy1 SenseDown - Servo2 Clockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M21);
    servo2 = servo2 + 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo2 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo2);
  }
  else if ( valY1 > 700 && valY2 < 600 && valY2 > 400)       //Joy1 SenseUp - Servo2 Counterclockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M22);
    servo2 = servo2 - 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo2 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo2);
  }

  /*===== M31 - M32 = SERVO 3 =====*/
  else if (valY2 > 700 && valY1 > 400 && valY1 < 600)             //Joy2 SenseDown - Servo3 Clockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M31);
    servo3 = servo3 + 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo3 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo3);
  }
  else if (valY2 < 350 && valY1 > 400 && valY1 < 600)       //Joy2 SenseUp - Servo3 Counterclockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M32);
    servo3 = servo3 - 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo3 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo3);
  }

  /*===== M41 - M42 = SERVO 4 =====*/
  if (valX1 > 700 && valX2 < 350)             //Joy1 SenseLeft 2 SenseRight - Servo4 Clockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M41);
    servo4 = servo4 + 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo4 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo4);
  }
  else if (valX1 < 350 && valX2 > 700)      //Joy1 SenseRight 2 SenseLeft - Servo4 Counterclockwise
  {
    delay(Speed);
    lcd.clear();
    BTserial.write(M42);
    servo4 = servo4 - 1;
    lcd.setCursor(0, 0);
    lcd.print("Servo4 Angle =");
    lcd.setCursor(0, 1);
    lcd.print(servo4);
  }
  limits();
}

////////////////////
//////INVERSE///////
////////////////////

void coordinateMode()
{
  valX1 = analogRead(joy1xPin);
  valY1 = analogRead(joy1yPin);
  valX2 = analogRead(joy2xPin);
  valY2 = analogRead(joy2yPin);

  /*==X VARIATION - JOYSTICK1 RIGHT/LEFT==*/
  if (valX1 > 700)            //X Increase - right
  {
    delay(Speed);
    BTserial.write(CX1);
    x = x + 1;
    lcd.clear();
  }
  else if (valX1 < 350)       //X Decrease - left
  {
    delay(Speed);
    BTserial.write(CX2);
    x = x - 1;
    lcd.clear();
  }

  /*==Y VARIATION - JOYSTICK1 UP/DOWN==*/
  if (valY1 > 700)            //Y increase - Up
  {
    delay(Speed);
    BTserial.write(CY1);
    y = y + 1;
    lcd.clear();
  }
  else if (valY1 < 350)       //Y Decrease - Down
  {
    delay(Speed);
    BTserial.write(CY2);
    y = y - 1;
    lcd.clear();
  }

  /*==Z BASE ROTATION==*/
  if (valX2 > 700)            //Base rotation clockwise
  {
    delay(Speed);
    BTserial.write(CZ1);
    base = base + 1;
    lcd.clear();
  }
  else if (valX2 < 350)       //Base rotation counterclockwise
  {
    delay(Speed);
    BTserial.write(CZ2);
    base = base - 1;
    lcd.clear();
  }

  /*==GRIP ANGLE VARIATION==*/
  if (valY2 > 700)            //Grip clockwise (+)
  {
    delay(Speed);
    BTserial.write(CA1);
    angle = angle - 1;
    lcd.clear();
  }
  else if (valY2 < 350)       //Grip counterclockwise (-)
  {
    delay(Speed);
    BTserial.write(CA2);
    angle = angle - 1;
    lcd.clear();
  }

  lcd.setCursor(0, 0);
  lcd.print("X");
  lcd.setCursor(1, 0);
  lcd.print(x);
  lcd.setCursor(6, 0);
  lcd.print("Y");
  lcd.setCursor(7, 0);
  lcd.print(y);
  lcd.setCursor(12, 0);
  lcd.print("Z=");
  lcd.setCursor(13, 0);
  lcd.print(base);
  lcd.setCursor(0, 1);
  lcd.print("GripAngle=");
  lcd.setCursor(10, 1);
  lcd.print(angle);
}

////////////////////
////POINT SAVE//////
////////////////////

void savePoint()
{
  if (DB == 0)      //Save point1
  {
    lcd.print("Saved point 1");
    DB = 1;
  }
  else if (DB == 1) //Save point2
  {
    lcd.print("Saved point 2");
    DB = 2;
  }
  else if (DB == 2) //Save point3
  {
    lcd.print("Saved point 3");
    DB = 3;
  }
  else if (DB == 3) //Save point4
  {
    lcd.print("Saved point 4");
    DB = 4;
  }
  else if (DB == 4) //Save point5
  {
    lcd.print("Saved point 5");
    DB = 5;
  }
  else if (DB == 5) //Save point Home
  {
    lcd.print("Last Point");
    lcd.setCursor(0, 1);
    lcd.print("= Home");
    DB = 6;
  }
  else if (DB == 6) //Max point allarm
  {
    lcd.print("MAX Points!!");
    lcd.setCursor(0, 1);
    lcd.print("click for reset...");
    DB = 7;
  }
  else if (DB == 7) //Reset poits memories
  {
    lcd.print("Reset points");
    lcd.setCursor(0, 1);
    lcd.print("Memories");
    DB = 0;
  }
}
