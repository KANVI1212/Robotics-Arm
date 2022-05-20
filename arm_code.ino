/*
  Project: Bracc.ino
  Date: 09/01/2020
  Last change: 20/01/2020

  Team: Basso Andrea
        Delmoro Niccol
        Gramaglia Giacomo
  //////ARM CODE//////
*/

//////////////////
///DECLERATIONS///
//////////////////

/*==LIBRARIES==*/
#include <VarSpeedServo.h>       //Servo library 
#include <SoftwareSerial.h>      //Virtual serial communication library, for bluetooth
SoftwareSerial BTserial(0, 1);   //RX (tx hc05) | TX (rx hc05-nodo)


/*==SERVO CONFIGURATIONS==*/
VarSpeedServo base;            //Base Rotation servo
int basePin = 3;
int baseAngle = 90;
const int baseInit = 90;
const int baseMin = 0;
const int baseMax = 180;

VarSpeedServo servo1;          //Shoulder 1 servo
int servo1Pin = 5;                //Pin attached to the servo
int servo1Angle = 73;             //Angle value of the servo
const int servo1Init = 73;        //Initial angle or in HomeMode
const int servo1Min = 0;          //Minimun angle
const int servo1Max = 180;        //Maximun angle

VarSpeedServo servo2;          //Elbow 2 servo
int servo2Pin = 6;
int servo2Angle = 26;
const int servo2Init = 26;
const int servo2Min = 0;
const int servo2Max = 180;

VarSpeedServo servo3;          //Wrist1 3 servo
const int servo3Pin = 9;
int servo3Angle = 147;
const int servo3Init = 147;
const int servo3Min = 0;
const int servo3Max = 180;

VarSpeedServo servo4;          //Wrist2 4 Servo
const int servo4Pin = 10;
int servo4Angle = 14;
const int servo4Init = 14;
const int servo4Min = 0;
const int servo4Max = 180;

VarSpeedServo gripper;        //Gripper servo - SG90
int gripperPin = 11;
int gripperAngle = 60;
int gripperInit = 60;
int gripperMax = 60;
int gripperMin = 0;

/*==BLUETOOTH COMMUNICATION COMMAND==*/
/*==DIRECT MOVEMENT COMMANDS==*/
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
/*==POINTS COMMAND==*/
char PRD = 'P';       //Reading point
/*==INVERSE KINEMATICS COMMANDS==*/
char CMM = '1';       //Turn in coordinate Mode
char CX1 = 'R';       //Increase X value
char CX2 = 'S';       //Decrease X value
char CY1 = 'T';       //Increase Y value
char CY2 = 'U';       //Decrease Y value
char CZ1 = 'V';       //Increase Z angle
char CZ2 = 'W';       //Decrease Z angle
char CA1 = 'X';       //Grip angle increase-clockwise
char CA2 = 'Y';       //Grip angle decrease-counterclockwise
/*==READING==*/
char reads = ' ';

/*==TIME AND SPEED==*/
int timeUp = 8;               //delay time
int servoSpeed;               //Speed servo general value
int servoSpeedSlow = 15;      //Slow movement speed
int servoSpeedFast = 80;      //Fast movement speed

/*==DIRECT POINTS==*/
int Dpoint1[6];          //Array for points, each contains servo angles value [base,servo1,servo2,servo3,servo4,gripper]
int Dpoint2[6];
int Dpoint3[6];
int Dpoint4[6];
int Dpoint5[6];
int DB;                  //Memory for save point
int timeSeq = 2000;      //Time after e sequence

/*==IK VALUES==*/
int XYMode;               //Value for current mode
int x;                    //X cartesian value
int xInit = 95;           //Initial x
int y;                    //Y cartesian value
int yInit = 80;           //Initial y
int gripAngle;            //Gripper angle angle, respect the x axis
int gripAngleInit = 90;   //Initiale angle
int IKpoint1[4];          //Array for points, each contains coordinate values [x,y,z,angle]
int IKpoint2[4];
int IKpoint3[4];
int IKpoint4[4];
int IKpoint5[4];
int DBIK;                 //Memory for save point

/*==IK CALCULATION VALUES==*/

//Links length
int link1 = 90;
int link2 = 85;
int link3 = 60;
int link4 = 142;
//Y offset between base and first joint
int offset = 93;

//1-Translation point G'
int yR;             //Y minus offset
double x2;          //G' x coordinate
double y2;          //G' y coordinate
double angle2;      //Complementar Grip Angle

//2-Slope calculation
double m;           //Pendance of the slope

//3-Circle intersacation
double a;
double b;
double c;
double xB1;
double xB2;
double xB;          //Intersacation x coordinate
double yB;          //Intersacation y coordinate
double r;           //Lenght of line from origin to B point

//4-Triangle angles
double alfa;        //Opposite triangle angle
double beta;        //B point triangle angle
double gamma;       //Origin triangle angle

//5-Deltoid angles
double d;           //Deltoid diagonal
double alfa2;       //Deltoid different angle
double delta;       //Deltoid equal angle value

//6-Angles values
double sum;         //Sum of the first angles
double J1;          //Joint1 Angle
double J2;          //Joint2 Angle
double J3;          //Joint3 Angle
double J4;          //Joint4 Angle

/////////////////
//////SETUP//////
/////////////////

void setup()
{
  //Begins
  BTserial.begin(38400);
  Serial.begin(9600);
  Serial.println("Ready for connection");

  //Servo attaching
  base.attach(basePin);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  gripper.attach(gripperPin);

  //Home angle initialitation
  moveHome();
}

/////////////////
//////LOOP///////
/////////////////

void loop()
{
  reading();
  limits();
  moveAll();
}

/////////////////
/////READING/////
/////////////////

void reading()
{
  reads = 0;

  //Bluetooth reading
  if (BTserial.available())
  {
    reads = BTserial.read();
  }
  if (reads == MDM)    //DIRECT MODE activation
  {
    XYMode = 0;
    moveHome();
  }
  else if (reads == CMM)    //INVERSE MODE activation
  {
    XYMode = 1;
    moveHome();

  }

  if (XYMode == 0)
    directMode();
  else if (XYMode == 1)
    coordinateMode();
}

void limits()           //Limit angle between angleMax and angle Min
{
  //Base limitation
  if (baseAngle > baseMax)
    baseAngle = baseMax;
  else if (baseAngle < baseMin)
    baseAngle = baseMin;

  //Servo1 limitation
  if (servo1Angle > servo1Max)
    servo1Angle = servo1Max;
  else if (servo1Angle < servo1Min)
    servo1Angle = servo1Min;

  //Servo2 limitation
  if (servo2Angle > servo2Max)
    servo2Angle = servo2Max;
  else if (servo2Angle < servo2Min)
    servo2Angle = servo2Min;

  //Servo3 limitation
  if (servo3Angle > servo3Max)
    servo3Angle = servo3Max;
  else if (servo3Angle < servo3Min)
    servo3Angle = servo3Min;

  //Servo4 limitation
  if (servo4Angle > servo4Max)
    servo4Angle = servo4Max;
  else if (servo4Angle < servo4Min)
    servo4Angle = servo4Min;

  //Gripper limitation
  if (gripperAngle > gripperMax)
    gripperAngle = gripperMax;
  else if (gripperAngle < gripperMin)
    gripperAngle = gripperMin;
}

/////////////////
//////MOVES//////
/////////////////

void moveAll()
{
  limits();
  //Base Movement
  base.write(baseAngle, servoSpeed, false);
  //Servo1 Movement
  servo1.write(servo1Angle, servoSpeed, false);
  //Servo2 Movement
  servo2.write(servo2Angle, servoSpeed, false);
  //Servo3 Movement
  servo3.write(servo3Angle, servoSpeed, false);
  //Servo4 Movement
  servo4.write(servo4Angle, servoSpeed, false);
  //Gripper Movement
  gripper.write(gripperAngle, servoSpeed, false);
}

void moveHome()
{
  servoSpeed = servoSpeedSlow;
  if (XYMode == 0)          //Direct Home
  {
    servoSpeed = servoSpeedSlow;
    baseAngle = baseInit;
    servo1Angle = servo1Init;
    servo2Angle = servo2Init;
    servo3Angle = servo3Init;
    servo4Angle = servo4Init;
    gripperAngle = gripperInit;
  }
  else if (XYMode == 1)     //Coordinate Home
  {
    x = xInit;
    y = yInit;
    gripAngle = gripAngleInit;
    baseAngle = baseInit;
    IKcalculation();
  }
  moveAll();
}

/////////////////
//////DKMODE/////
/////////////////

void directMode()
{
  /*=====Base reading=====*/
  if (reads == MB1)        //Base Clockwise
  {
    baseAngle = baseAngle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == MB2)   //Base Counterclockwise
  {
    baseAngle = baseAngle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Servo1 reading=====*/
  if (reads == M11)        //S1 Clockwise
  {
    servo1Angle = servo1Angle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == M12)   //S1 Counterclockwise
  {
    servo1Angle = servo1Angle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Servo2 reading=====*/
  //Servo2 reading
  if (reads == M21)        //S2 Clockwise
  {
    servo2Angle = servo2Angle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == M22)   //S2 Counterclockwise
  {
    servo2Angle = servo2Angle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Servo3 reading=====*/
  if (reads == M31)       //S3 Clockwise
  {
    servo3Angle = servo3Angle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == M32)  //S3 Counterclockwise
  {
    servo3Angle = servo3Angle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Servo4 reading=====*/
  if (reads == M41)       //S4 Clockwise
  {
    servo4Angle = servo4Angle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == M42)  //S4 Counterclockwise
  {
    servo4Angle = servo4Angle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Gripper reading=====*/
  if (reads == MG1)       //Opening
  {
    gripperAngle = 60;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == MG2)  //Closing
  {
    gripperAngle = 0;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Homing reading=====*/
  if (reads == MHO)       //Set angles to home position
  {
    delay(timeUp);
    moveHome();
  }

  /*=====Reading points DIRECT=====*/
  if (reads == PRD)       //Points reading
  {
    savePoint();
  }

  /*=====Points sequence DIRECT=====*/
  if (reads == MPS)
  {
    moveSequence();
  }
}

//////////////////POINTS DIRECT///////////////////////////

void   savePoint()
{
  if (DB == 0)        //Save point1
  {
    Dpoint1[0] = base.read();
    Dpoint1[1] = servo1.read();
    Dpoint1[2] = servo2.read();
    Dpoint1[3] = servo3.read();
    Dpoint1[4] = servo4.read();
    Dpoint1[5] = gripper.read();
    DB = 1;
  }
  else if (DB == 1)   //Save point2
  {
    Dpoint2[0] = base.read();
    Dpoint2[1] = servo1.read();
    Dpoint2[2] = servo2.read();
    Dpoint2[3] = servo3.read();
    Dpoint2[4] = servo4.read();
    Dpoint2[5] = gripper.read();
    DB = 2;
  }
  else if (DB == 2)   //Save point3
  {
    Dpoint3[0] = base.read();
    Dpoint3[1] = servo1.read();
    Dpoint3[2] = servo2.read();
    Dpoint3[3] = servo3.read();
    Dpoint3[4] = servo4.read();
    Dpoint3[5] = gripper.read();
    DB = 3;
  }
  else if (DB == 3)   //Save point4
  {
    Dpoint4[0] = base.read();
    Dpoint4[1] = servo1.read();
    Dpoint4[2] = servo2.read();
    Dpoint4[3] = servo3.read();
    Dpoint4[4] = servo4.read();
    Dpoint4[5] = gripper.read();
    DB = 4;
  }
  else if (DB == 4)   //Save point5
  {
    Dpoint5[0] = base.read();
    Dpoint5[1] = servo1.read();
    Dpoint5[2] = servo2.read();
    Dpoint5[3] = servo3.read();
    Dpoint5[4] = servo4.read();
    Dpoint5[5] = gripper.read();
    DB = 5;
  }
  else if (DB == 5)   //Save Point Home
  {
    DB = 6;
  }
  else if (DB == 6)  //Max point allert
  {
    delay(1000);
  }
  else if (DB == 7)  //Reset Points
  {
    DB == 0;
  }
}

void moveSequence()
{
  servoSpeed = servoSpeedSlow;
  if (DB == 6 || DB == 7)          //5 points sequence + Home
  {
    Point1();
    moveAll();
    delay(timeSeq);
    Point2();
    moveAll();
    delay(timeSeq);
    Point3();
    moveAll();
    delay(timeSeq);
    Point4();
    moveAll();
    delay(timeSeq);
    Point5();
    moveAll();
    delay(timeSeq);
    moveHome();
    delay(timeSeq);
  }
  else if (DB == 5)     //5 points sequence
  {
    Point1();
    moveAll();
    delay(timeSeq);
    Point2();
    moveAll();
    delay(timeSeq);
    Point3();
    moveAll();
    delay(timeSeq);
    Point4();
    moveAll();
    delay(timeSeq);
    Point5();
    moveAll();
    delay(timeSeq);
  }
  else if (DB == 4)                        //4 points sequence
  {
    Point1();
    moveAll();
    delay(timeSeq);
    Point2();
    moveAll();
    delay(timeSeq);
    Point3();
    moveAll();
    delay(timeSeq);
    Point4();
    moveAll();
    delay(timeSeq);
  }
  else if (DB == 3)                        //3 points sequence
  {
    Point1();
    moveAll();
    delay(timeSeq);
    Point2();
    moveAll();
    delay(timeSeq);
    Point3();
    moveAll();
    delay(timeSeq);
  }
  else if (DB == 2)                        //2 points sequence
  {
    Point1();
    moveAll();
    delay(timeSeq);
    Point2();
    moveAll();
    delay(timeSeq);
  }
  else if (DB == 1)                        //1 point sequence
  {
    Point1();
    moveAll();
    delay(timeSeq);
  }
}

void Point1()
{
  baseAngle = Dpoint1[0];
  servo1Angle = Dpoint1[1];
  servo2Angle = Dpoint1[2];
  servo3Angle = Dpoint1[3];
  servo4Angle = Dpoint1[4];
  gripperAngle = Dpoint1[5];
}
void Point2()
{
  baseAngle = Dpoint2[0];
  servo1Angle = Dpoint2[1];
  servo2Angle = Dpoint2[2];
  servo3Angle = Dpoint2[3];
  servo4Angle = Dpoint2[4];
  gripperAngle = Dpoint2[5];
}
void Point3()
{
  baseAngle = Dpoint3[0];
  servo1Angle = Dpoint3[1];
  servo2Angle = Dpoint3[2];
  servo3Angle = Dpoint3[3];
  servo4Angle = Dpoint3[4];
  gripperAngle = Dpoint3[5];
}
void Point4()
{
  baseAngle = Dpoint4[0];
  servo1Angle = Dpoint4[1];
  servo2Angle = Dpoint4[2];
  servo3Angle = Dpoint4[3];
  servo4Angle = Dpoint4[4];
  gripperAngle = Dpoint4[5];
}
void Point5()
{
  baseAngle = Dpoint5[0];
  servo1Angle = Dpoint5[1];
  servo2Angle = Dpoint5[2];
  servo3Angle = Dpoint5[3];
  servo4Angle = Dpoint5[4];
  gripperAngle = Dpoint5[5];
}

/////////////////
//////IKMODE/////
/////////////////

void coordinateMode()
{
  /*=====X VALUE VARIATION=====*/
  if (reads == CX1)           //X increase
  {
    x = x + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == CX2)      //X decrease
  {
    x = x - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Y VALUE VARIATION=====*/
  if (reads == CY1)           //Y increase
  {
    y = y + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == CY2)      //Y decrease
  {
    y = y - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====Z ROTATION VARIATION=====*/
  if (reads == CZ1)           //Z clockwise rotation
  {
    baseAngle = baseAngle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == CZ2)      //Z counterclockwise rotation
  {
    baseAngle = baseAngle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====GRIP ANGLE VARIATION=====*/
  if (reads == CA1)           //GripAngle increase
  {
    gripAngle = gripAngle + 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == CA2)      //GripAngle decrease
  {
    gripAngle = gripAngle - 1;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====HOME POSITION IK=====*/
  if (reads == MHO)
  {
    moveHome();
  }

  /*=====GRIPPER READING=====*/
  if (reads == MG1)           //Opening
  {
    gripperAngle = 60;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }
  else if (reads == MG2)      //Closing
  {
    gripperAngle = 0;
    delay(timeUp);
    servoSpeed = servoSpeedFast;
  }

  /*=====IK READING POINTS=====*/
  if (reads == PRD)
  {
    savePointIK();
  }
  if (reads == MPS)
  {
    moveSequenceIK();
  }
  IKcalculation();
}

//////////////////POINTS INVERSE///////////////////////////
void savePointIK()
{
  if (DBIK == 0)        //Save point1
  {
    IKpoint1[0] = x;
    IKpoint1[1] = y;
    IKpoint1[2] = gripAngle;
    IKpoint1[3] = base.read();
    DBIK = 1;
  }
  else if (DBIK == 1)        //Save point2
  {
    IKpoint1[0] = x;
    IKpoint1[1] = y;
    IKpoint1[2] = gripAngle;
    IKpoint1[3] = base.read();
    DBIK = 2;
  }
  else if (DBIK == 2)        //Save point3
  {
    IKpoint1[0] = x;
    IKpoint1[1] = y;
    IKpoint1[2] = gripAngle;
    IKpoint1[3] = base.read();
    DBIK = 3;
  }
  else if (DBIK == 3)        //Save point4
  {
    IKpoint1[0] = x;
    IKpoint1[1] = y;
    IKpoint1[2] = gripAngle;
    IKpoint1[3] = base.read();
    DBIK = 4;
  }
  else if (DBIK == 4)        //Save point5
  {
    IKpoint1[0] = x;
    IKpoint1[1] = y;
    IKpoint1[2] = gripAngle;
    IKpoint1[3] = base.read();
    DBIK = 5;
  }
  else if (DBIK == 5)        //Save point Home at last
  {
    DBIK = 6;
  }
  else if (DBIK == 6)        //Max point allert
  {
    DBIK = 7;
    delay(500);
  }
  else if (DBIK == 7)        //Reset points
  {
    DBIK = 0;
    delay(1000);
  }
}

void moveSequenceIK()
{
  servoSpeed = servoSpeedSlow;
  if (DBIK == 7 || DBIK == 6)  //Point 1-2-3-4-5-Home Sequence
  {
    IKPoint1();
    delay(timeSeq);
    IKPoint2();
    delay(timeSeq);
    IKPoint3();
    delay(timeSeq);
    IKPoint4();
    delay(timeSeq);
    IKPoint5();
    delay(timeSeq);
    moveHome();
    delay(timeSeq);
  }
  else if (DBIK == 5)          //Point 1-2-3-4-5 Sequence
  {
    IKPoint1();
    delay(timeSeq);
    IKPoint2();
    delay(timeSeq);
    IKPoint3();
    delay(timeSeq);
    IKPoint4();
    delay(timeSeq);
    IKPoint5();
    delay(timeSeq);
  }
  else if (DBIK == 4)          //Point 1-2-3-4 Sequence
  {
    IKPoint1();
    delay(timeSeq);
    IKPoint2();
    delay(timeSeq);
    IKPoint3();
    delay(timeSeq);
    IKPoint4();
    delay(timeSeq);
  }
  else if (DBIK == 3)          //Point 1-2-3 Sequence
  {
    IKPoint1();
    delay(timeSeq);
    IKPoint2();
    delay(timeSeq);
    IKPoint3();
    delay(timeSeq);
  }
  else if (DBIK == 2)          //Point 1-2 Sequence
  {
    IKPoint1();
    delay(timeSeq);
    IKPoint2();
    delay(timeSeq);
  }
  else if (DBIK == 1)          //Point 1 Sequence
  {
    IKPoint1();
    delay(timeSeq);
  }
}

int Vx;
int Vy;
double pendance;
int Vz;
int Vangle;
double steps;
double q;

void IKPoint1()
{
  Vx = IKpoint1[0] - x;
  Vy = IKpoint1[1] - y;
  Vangle = IKpoint1[2] - gripAngle;
  Vz = IKpoint1[3] - baseAngle;

  pendance = Vy / Vx;
  q = y - (pendance * x);

  if (Vx > 0)
  {
    for (steps = 0; steps < Vx; steps++)
    {
      x++;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint1[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint1[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint1[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint1[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
  if (Vx < 0)
  {
    for (steps = Vx; steps > 0; steps--)
    {
      x--;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint1[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint1[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint1[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint1[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
}
void IKPoint2()
{
  Vx = IKpoint2[0] - x;
  Vy = IKpoint2[1] - y;
  Vangle = IKpoint2[2] - gripAngle;
  Vz = IKpoint2[3] - baseAngle;

  pendance = Vy / Vx;
  q = y - (pendance * x);

  if (Vx > 0)
  {
    for (steps = 0; steps < Vx; steps++)
    {
      x++;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint2[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint2[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint2[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint2[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
  if (Vx < 0)
  {
    for (steps = Vx; steps > 0; steps--)
    {
      x--;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint2[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint2[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint2[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint2[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
}
void IKPoint3()
{
  Vx = IKpoint3[0] - x;
  Vy = IKpoint3[1] - y;
  Vangle = IKpoint3[2] - gripAngle;
  Vz = IKpoint3[3] - baseAngle;

  pendance = Vy / Vx;
  q = y - (pendance * x);

  if (Vx > 0)
  {
    for (steps = 0; steps < Vx; steps++)
    {
      x++;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint3[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint3[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint3[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint3[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
  if (Vx < 0)
  {
    for (steps = Vx; steps > 0; steps--)
    {
      x--;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint3[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint3[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint3[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint3[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
}
void IKPoint4()
{
  Vx = IKpoint4[0] - x;
  Vy = IKpoint4[1] - y;
  Vangle = IKpoint4[2] - gripAngle;
  Vz = IKpoint4[3] - baseAngle;

  pendance = Vy / Vx;
  q = y - (pendance * x);

  if (Vx > 0)
  {
    for (steps = 0; steps < Vx; steps++)
    {
      x++;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint4[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint4[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint4[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint4[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
  if (Vx < 0)
  {
    for (steps = Vx; steps > 0; steps--)
    {
      x--;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint4[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint4[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint4[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint4[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
}
void IKPoint5()
{
  Vx = IKpoint5[0] - x;
  Vy = IKpoint5[1] - y;
  Vangle = IKpoint5[2] - gripAngle;
  Vz = IKpoint5[3] - baseAngle;

  pendance = Vy / Vx;
  q = y - (pendance * x);

  if (Vx > 0)
  {
    for (steps = 0; steps < Vx; steps++)
    {
      x++;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint5[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint5[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint5[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint5[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
  if (Vx < 0)
  {
    for (steps = Vx; steps > 0; steps--)
    {
      x--;
      y = (pendance * x) + q;
      if (Vz > 0)
        if (baseAngle < IKpoint5[3])
          baseAngle++;
      if (Vz < 0)
        if (baseAngle > IKpoint5[3])
          baseAngle--;
      if (Vangle > 0)
        if (gripAngle < IKpoint5[2])
          gripAngle++;
      if (Vangle < 0)
        if (gripAngle > IKpoint5[2])
          gripAngle--;
      delay(100);
      IKcalculation();
      moveAll();
    }
  }
}

//////////////////
//IK CALCULATION//
//////////////////

void IKcalculation()
{
  yR = y - offset;

  //1- Translation to G' point
  angle2 = 180 - gripAngle;
  x2 = x + (link4 * cos(angle2 * (PI / 180.00)));
  y2 = yR + (link4 * sin(angle2 * (PI / 180.00)));

  //2- Slope calculation
  m = y2 / x2;

  //3- Circle intersacation
  a = 1.00 + sq(m);
  b = -(2.00 * x2) - (2.00 * m * y2);
  c = sq(x2) + sq(y2) - sq(link3);
  xB1 = (-b + sqrt(sq(b) - (4 * a * c))) / (2.00 * a);
  xB2 = (-b - sqrt(sq(b) - (4 * a * c))) / (2.00 * a);
  xB = min(xB1, xB2);
  yB = m * xB;
  r = sqrt(sq(xB) + sq(yB));

  //4- Triangle angles
  alfa = (acos((sq(link1) + sq(link2) - sq(r)) / (2.00 * link1 * link2))) * (180.00 / PI);
  beta = (acos((sq(link2) + sq(r) - sq(link1)) / (2.00 * link2 * r))) * (180.00 / PI);
  gamma = (acos((sq(link1) + sq(r) - sq(link2)) / (2.00 * link1 * r))) * (180.00 / PI);

  //5- Deltoid angles
  delta = (180.00 - beta) * (PI / 180.00);
  d = sqrt(sq(link2) + sq(link3) - (2.00 * link2 * link3 * cos(delta)));
  alfa2 = 2.00 * ((asin((link3 * sin(delta)) / d)) * (180.00 / PI));

  //6- Joints angles calculation
  J1 = gamma + ((atan(y2 / x2)) * (180.00 / PI));
  J2 = alfa - 90.00 + alfa2;
  J3 = delta * (180.00 / PI) - 90.00;
  sum = J1 + alfa + alfa2 + delta * (180 / PI);
  J4 = 270 + angle2 - sum;

  //7- Servo angles convertion
  servo1Angle = 180 - J1;
  servo2Angle = J2;
  servo3Angle = 180 - J3;
  servo4Angle = J4;
}
