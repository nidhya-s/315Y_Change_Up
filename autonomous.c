#include "main.h"

#include "config.h"

#define RIGHTANGLE 780

#define TURNENCODERPRECISION 6

#define MOVEENCODERPRECISION 7

#define NUMDRIVEMOTORS 6

#define VELOCITYLIMIT 3

#define GYRODRIFTRATE 0.0

#define KP 0.10   //0.12

#define KI 0.0025 //0.003

#define INTEGRALLIMIT 6

#define DRIVEP 0.100

#define DRIVEI 0.0018

#define DRIVED 0.055

#define CORRD 0.3 //0.75

#define TDRIVEP 0.145 //.130

#define TDRIVEI 0.0089

#define TDRIVED 0.07 //0.04

#define DRIVEINTEGRALLIMIT 1000

#define DRIVEMAXVEL 1

#define DRIVEPOSTOL 50

#define TDRIVEINTEGRALLIMIT 1000

#define TDRIVEMAXVEL 5

#define TDRIVEPOSTOL 150 //6

#define STARTHEADING 900

#define GYRO_START 0

#define TOPSENSOR 2400

//#define TOPSENSORINDEX 2400

#define BOTTOMSENSOR 2700

#define CORNERGOALTIMEOUT 2000

#define MIDDLEGOALTIMEOUT 2000

bool run_intake = false;

int intakeTimeout = 0;

bool run_flywheel = false;

int flywheelTimeout = 0;

int currentheading = 0;

uint32_t programStartTime = 0;

int  imu_sensor = 1;

adi_gyro_t gyro;

extern void middleGoal(bool auton, int timeout);

extern void middleGoalOneRed(bool auton, int timeout);

extern void cornerGoal(bool auton, int timeout);

extern void cornerGoalOneRed(bool auton);

#define max(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

void clearDriveMotors(){
    motor_tare_position(PORT_DRIVELEFTFRONT);
    motor_tare_position(PORT_DRIVELEFTMIDDLE);
    motor_tare_position(PORT_DRIVELEFTBACK);
    motor_tare_position(PORT_DRIVERIGHTFRONT);
    motor_tare_position(PORT_DRIVERIGHTMIDDLE);
    motor_tare_position(PORT_DRIVERIGHTBACK);
}

void assignDriveMotorsAuton(int power){
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVELEFTMIDDLE, power);
    motor_move(PORT_DRIVERIGHTFRONT, power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTMIDDLE, power);
    motor_move(PORT_DRIVERIGHTBACK, power);
}

void assignDriveMotorsPower(int rightSide, int leftSide){
    motor_move(PORT_DRIVELEFTFRONT, leftSide);
    motor_move(PORT_DRIVELEFTMIDDLE, leftSide);
    motor_move(PORT_DRIVELEFTBACK, leftSide);
    motor_move(PORT_DRIVERIGHTFRONT, rightSide);
    motor_move(PORT_DRIVERIGHTMIDDLE, rightSide);
    motor_move(PORT_DRIVERIGHTBACK, rightSide);
}

double averageVelocity(){
    return (abs(motor_get_actual_velocity(PORT_DRIVELEFTFRONT)) + abs(motor_get_actual_velocity(PORT_DRIVELEFTBACK)) +
        abs(motor_get_actual_velocity(PORT_DRIVERIGHTFRONT)) + abs(motor_get_actual_velocity(PORT_DRIVERIGHTBACK)) +
        abs(motor_get_actual_velocity(PORT_DRIVELEFTMIDDLE)) + abs(motor_get_actual_velocity(PORT_DRIVERIGHTMIDDLE)))/
        NUMDRIVEMOTORS;
}

void resetGyro(){
    adi_gyro_reset(gyro);
    imu_reset(IMU_PORT);
}

void getHeading(){
  int wrap = 0;
  int imucur = STARTHEADING;
  while(true){
      int previousHeading = imucur;
      imucur = imu_get_heading(IMU_PORT)*10 + STARTHEADING;
      if(previousHeading - imucur > 1000)
      {
        wrap ++;
      }
      else if(imucur - previousHeading > 1000)
      {
        wrap --;
      }
      currentheading = imucur + 3600*wrap;
      delay(5);
  }

      //return (heading + 1800) % 3600 - 1800;
}

void assignDriveMotorsDist(int leftSide, int rightSide, int power, bool clear, bool turn){
    if (clear)
    {
        clearDriveMotors();
    }

    int currentPrecision = MOVEENCODERPRECISION;
    if (turn)
    {
      currentPrecision = TURNENCODERPRECISION;
    }

    motor_move_absolute(PORT_DRIVELEFTFRONT, leftSide, power);
    motor_move_absolute(PORT_DRIVELEFTMIDDLE, leftSide, power);
    motor_move_absolute(PORT_DRIVELEFTBACK, leftSide, power);
    motor_move_absolute(PORT_DRIVERIGHTFRONT, rightSide, power);
    motor_move_absolute(PORT_DRIVERIGHTMIDDLE, rightSide, power);
    motor_move_absolute(PORT_DRIVERIGHTBACK, rightSide, power);
    delay(100);
    uint32_t timeout = millis();
    while ((abs(motor_get_position(PORT_DRIVELEFTFRONT) - leftSide) + abs(motor_get_position(PORT_DRIVELEFTBACK) - leftSide) + abs(motor_get_position(PORT_DRIVELEFTMIDDLE) - leftSide) +
      (abs(motor_get_position(PORT_DRIVERIGHTFRONT) - rightSide) + abs(motor_get_position(PORT_DRIVERIGHTBACK) - rightSide)) + abs(motor_get_position(PORT_DRIVERIGHTMIDDLE) - rightSide)) >
      currentPrecision * NUMDRIVEMOTORS) // ||
                                                  //(turn && (averageVelocity() >= VELOCITYLIMIT)))
    {
        uint32_t ctime = millis();
        if(ctime - timeout > 1100){
          break;
        }
        delay(20);
    }

    delay(250);
    assignDriveMotorsPower(0, 0);
}

void straightPID(int leftSide, int rightSide, int power, int heading){
    leftSide *= 200.0 / 257.0;
    rightSide *= 200.0 / 257.0;
    power *= 200.0 / 257.0;

    if (left)
    {
        heading = -heading;
    }

    clearDriveMotors();

    double curP = DRIVEP;
    double curI = DRIVEI;
    double curD = DRIVED;
    double CORR = CORRD;

    int leftErrorChange = 0;
    int rightErrorChange = 0;
    int leftError = 3 * leftSide;
    int rightError = 3 * rightSide;
    int prevLeftError = 3 * leftSide;
    int prevRightError = 3 * rightSide;
    int leftIntegral = 0;
    int rightIntegral = 0;

    bool endSequence = false;
    int startEnd = 0;
    int endTime = 500;

    while (abs(leftErrorChange) + abs(rightErrorChange) > DRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime)))
    {
           if (abs(leftError) + abs(rightError) <= DRIVEPOSTOL && !endSequence)
           {
              endSequence = true;
              startEnd = millis();
           }

           leftIntegral += leftError;
           rightIntegral += rightError;

           if (leftIntegral > DRIVEINTEGRALLIMIT)
           {
              leftIntegral = DRIVEINTEGRALLIMIT;
           }
           if (leftIntegral < -DRIVEINTEGRALLIMIT)
           {
              leftIntegral = -DRIVEINTEGRALLIMIT;
           }
           if (rightIntegral > DRIVEINTEGRALLIMIT)
           {
              rightIntegral = DRIVEINTEGRALLIMIT;
           }
           if (rightIntegral < -DRIVEINTEGRALLIMIT)
           {
              rightIntegral = -DRIVEINTEGRALLIMIT;
           }

           leftErrorChange = leftError - prevLeftError;
           rightErrorChange = rightError - prevRightError;

           prevLeftError = leftError;
           prevRightError = rightError;

           int rightCorrection = heading - currentheading;
           assignDriveMotorsPower(min(power, max(-power, leftError * DRIVEP + leftIntegral * DRIVEI + leftErrorChange * DRIVED + rightCorrection * CORR)), min(power, max(-power, rightError * DRIVEP + rightIntegral * DRIVEI + rightErrorChange * DRIVED - rightCorrection * CORR)));

           delay(5);

           leftError = 3 * leftSide - motor_get_position(PORT_DRIVELEFTFRONT) - motor_get_position(PORT_DRIVELEFTBACK) - motor_get_position(PORT_DRIVELEFTMIDDLE);
           rightError = 3 * rightSide - motor_get_position(PORT_DRIVERIGHTFRONT) - motor_get_position(PORT_DRIVERIGHTBACK) - motor_get_position(PORT_DRIVERIGHTMIDDLE);
        }
        assignDriveMotorsPower(0, 0);
     }

void turnGyro(int degrees, int power){
        uint32_t cur = millis();
        if (left)
        {
           degrees = -degrees;
        }

        double curP = TDRIVEP;
        double curI = TDRIVEI;
        double curD = TDRIVED;

        int errorChange = 0;
        int error = degrees - currentheading;
        int prevError = degrees - currentheading;
        int integral = 0;

        bool endSequence = false;
        int startEnd = 0;
        int endTime = 200; //500
                                                                                                                        //1155, 955
        while ((abs(errorChange) > TDRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime))) && millis() - cur < 800)
        {
           if (abs(error) <= TDRIVEPOSTOL && !endSequence)
           {
              endSequence = true;
              startEnd = millis();
           }

           integral += error;
           if (integral > TDRIVEINTEGRALLIMIT)
           {
              integral = TDRIVEINTEGRALLIMIT;
           }
           else if (integral < -TDRIVEINTEGRALLIMIT)
           {
              integral = -TDRIVEINTEGRALLIMIT;
           }

           errorChange = error - prevError;
           prevError = error;

           delay(5);

           error = degrees - currentheading;
           int powerToAssign = min(power, max(-power, error * TDRIVEP + integral * TDRIVEI + errorChange * TDRIVED));
           assignDriveMotorsPower(powerToAssign, -powerToAssign);
        }
        assignDriveMotorsPower(0, 0);
     }

void coast(int ticks, int power, int heading, bool corr){
        ticks *= 200.0 / 257.0;
        power *= 200.0 / 257.0;
        if (left){
           heading = -heading;
        }

        int error = 0;

        clearDriveMotors();
        assignDriveMotorsPower(power, power);
        while (abs(motor_get_position(PORT_DRIVELEFTFRONT)) + abs(motor_get_position(PORT_DRIVELEFTBACK)) + abs(motor_get_position(PORT_DRIVELEFTMIDDLE)) +
                   abs(motor_get_position(PORT_DRIVERIGHTFRONT)) + abs(motor_get_position(PORT_DRIVERIGHTBACK)) + abs(motor_get_position(PORT_DRIVERIGHTMIDDLE))<
               ticks * NUMDRIVEMOTORS)
        {
           error = heading - currentheading;
           if (!corr){
              error = 0;
           }
           assignDriveMotorsPower(power + error * CORRD, power - error * CORRD);
           //assignDriveMotorsPower(30, 30);

           delay(5);
        }
        assignDriveMotorsPower(0, 0);
     }

void forward(int ticks, int power, int heading){
        straightPID(ticks, ticks, power, heading);
     }

void backward(int ticks, int power, int heading){
        straightPID(-ticks, -ticks, power, heading);
     }

void forwardCoast(int ticks, int power, int heading){
        coast(ticks, power, heading, true);
     }

void backwardCoast(int ticks, int power, int heading){
        coast(ticks, -power, heading, true);
     }

void turnLeft(int degrees, int power){
        turnGyro(degrees, power);
     }

void turnRight(int degrees, int power){
        turnGyro(degrees, power);
     }

void turnLeftNOT(int power, int time){
         motor_move(PORT_DRIVELEFTFRONT, power);
         motor_move(PORT_DRIVELEFTMIDDLE, power);
         motor_move(PORT_DRIVERIGHTFRONT, -power);
         motor_move(PORT_DRIVERIGHTMIDDLE, -power);
         motor_move(PORT_DRIVELEFTBACK, power);
         motor_move(PORT_DRIVERIGHTBACK, -power);
         delay(time);
         assignDriveMotorsAuton(0);
     }

void turnRightNOT(int power, int time){
         motor_move(PORT_DRIVELEFTFRONT, -power);
         motor_move(PORT_DRIVELEFTMIDDLE, -power);
         motor_move(PORT_DRIVERIGHTFRONT, power);
         motor_move(PORT_DRIVELEFTBACK, -power);
         motor_move(PORT_DRIVERIGHTMIDDLE, power);
         motor_move(PORT_DRIVERIGHTBACK, power);
         delay(time);
         assignDriveMotorsAuton(0);
     }

void turnRD(int ticks, int power, bool clear){
          assignDriveMotorsDist(-ticks, ticks, power, clear, true);
     }

void turnLD(int ticks, int power, bool clear){
          assignDriveMotorsDist(ticks, -ticks, power, clear, true);
     }
//checks if there is a white line
bool isWhiteLine(int threshold){
       int left = adi_analog_read_calibrated(LINE_TRACKER_LEFT);
       int middle = adi_analog_read_calibrated(LINE_TRACKER_MIDDLE);
       int right = adi_analog_read_calibrated(LINE_TRACKER_RIGHT);
       int avrg = (left+right+middle)/3;
       if (avrg < threshold)
       {
         return true;
       }
       return false;
     }

void autonRollers(int power){
  motor_move(PORT_ROLLERS, -power);
}

void autonFlywheel(int power){
  motor_move(PORT_FLYWHEEL, power);
}

void brake(int power){
  assignDriveMotorsAuton(power);
  delay(150);
  assignDriveMotorsAuton(0);
}

void earlyAuton3Balls(){
  autonRollers(127);
  assignDriveMotorsAuton(50);
  while(!isWhiteLine(1200)) {
    delay(30);
  }
  //brake(-20);
  brake(-40);
  turnRight(1250, 75);
  delay(100);

  forwardCoast(1100, 70, 1250);
  /*
  assignDriveMotorsAuton(60);
  delay(900);
  assignDriveMotorsAuton(0);
  */

  //autonRollers(0);
  autonFlywheel(127);
  delay(700);
  autonFlywheel(0);
  autonRollers(0);
  delay(100);

  autonRollers(127);
  assignDriveMotorsAuton(-50);
  while(!isWhiteLine(1200)) {
    delay(30);
  }
  //brake(-20);
  brake(40);
  turnRight(900, 75);
  delay(100);
  backwardCoast(2500, 70, 900);
  brake(40);
/*
  delay(200);
  autonFlywheel(127);
  delay(140);
  autonFlywheel(0);
  autonRollers(0);
  */
  turnLeft(1800, 75);
  delay(100);
  forwardCoast(250, 70, 1800);
  /*
  assignDriveMotorsAuton(60);
  delay(700);
  assignDriveMotorsAuton(0);
*/
  autonFlywheel(127);
  delay(700);
  autonFlywheel(0);
  delay(100);

//maybe check for gray

  backwardCoast(600, 70, 1800);
  brake(40);
  turnRight(2700, 75);
  delay(100);
  forwardCoast(2700, 70, 2700);
  brake(40);

//changed distance back away at end and turn angle from 2350
  turnLeft(2250, 75);
  autonRollers(127);
  autonFlywheel(127);
  delay(100);
  forwardCoast(1100, 70, 2250);
  /*
  assignDriveMotorsAuton(60);
  delay(900);
  assignDriveMotorsAuton(0);
*/
  //autonFlywheel(127);
  delay(1300);
  autonFlywheel(0);
  autonRollers(0);
  //delay(200);
  backwardCoast(800, 70, 2250);
}

void threeBallAuton(bool left){
  autonRollers(127);
  assignDriveMotorsAuton(75);
  delay(400);
  assignDriveMotorsAuton(0);
  if(left){
    turnLeft(1100, 75);
  }
  else{
    turnRight(1100, 75);
  }
  delay(200);

  autonFlywheel(127);
  delay(700);
  /*autonRollers(127);
  forwardCoast(1400, 60, -1250);
  delay(500);
  autonRollers(0);

  autonFlywheel(127);
  delay(700);
  autonFlywheel(0);
  delay(200);

  backwardCoast(700, 60, -1250);

  delay(200);
  if(left){
    turnRight(900, 75);
  }
  else{
    turnLeft(900, 75);
  }
  delay(200);

  autonRollers(127);
  backwardCoast(2900, 100, -900);
  brake(20);
  delay(200);

  if(left){
    turnLeft(1800, 75);
  }
  else{
    turnRight(1800, 75);
  }
  delay(200);
  forwardCoast(100, 60, -1800);
  autonFlywheel(127);*/
}

void flipout(){
  autonFlywheel(127);
  delay(150);
  autonFlywheel(0);

}

void indexBall(){
  autonFlywheel(127);
  int timeout = millis();
  while(adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP) > TOPSENSOR && run_flywheel)
  {
    if(flywheelTimeout != 0 && millis() - timeout >= flywheelTimeout)
    {
      break;
    }
    autonFlywheel(127);
    delay(20);
  }
  flywheelTimeout = 0;
  delay(50);
  autonFlywheel(0);
}

void intakeBall(){
  autonRollers(127);
  int timeout = millis();
  while(adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM) > BOTTOMSENSOR && run_intake)
  {
    if(intakeTimeout != 0 && millis() - timeout >= intakeTimeout)
    {
      break;
    }
    autonRollers(127);
    delay(20);
  }
  intakeTimeout = 0;
  delay(50);
  autonRollers(0);
}

void runIntake(){
  while(true)
  {
    if(run_intake)
    {
      intakeBall();
    }
    run_intake = false;
    delay(50);
  }
}

void runFlywheel(){
  while(true)
  {
    if(run_flywheel)
    {
      indexBall();
    }
    run_flywheel = false;
    delay(50);
  }
}

void setFlywheel(int timeout, bool wait){
  run_flywheel = true;
  flywheelTimeout = timeout;
}

void setRollers(int timeout, bool wait){
  run_intake = true;
  intakeTimeout = timeout;
}

void holdPower(bool on){
  if(on){
    assignDriveMotorsAuton(20);
  }
  else{
    assignDriveMotorsAuton(0);
  }
}

void firstHalf(){
  //spit out one ball
  autonRollers(127);
  setFlywheel(0, false);
  backwardCoast(200, 127, -1275);
  autonRollers(0);
  delay(50);
  autonRollers(-127);
  //delay(200);
  backwardCoast(700, 127, -1275);

  backwardCoast(800, 127, -1275);
  autonRollers(0);
  delay(200);
  autonRollers(127);

  if(left){
    turnRight(-900, 110);
  }
  else{
    turnLeft(-900, 110);
  }

  forwardCoast(1600, 115, -900);
  setRollers(0, false);
  delay(300);
  run_flywheel = false;
  //Second Goal
  //turn to pick up ball
  backwardCoast(400, 127, -900);
  brake(20);

  //pick up other ball
  if(left){
    turnLeft(780, 110);
  }
  else{
    turnRight(780, 110);
  }
  //eject ball
  autonFlywheel(-127);
  forwardCoast(1500, 127, 780);
  autonFlywheel(0);

  autonRollers(127);
  setFlywheel(0, true);
  forwardCoast(1000, 127, 780);
  setFlywheel(0, true);
  forwardCoast(1130, 100, 780);
  setRollers(0, false);
  brake(-30);

  //shoot second goal
  if(left){
    turnRight(1800, 110);
  }
  else{
    turnLeft(1800, 110);
  }
  //run_flywheel = true;
  //setRollers(0, false);
  forwardCoast(1800, 115, 1800);
  assignDriveMotorsAuton(10);
  middleGoal(true, MIDDLEGOALTIMEOUT);
  assignDriveMotorsAuton(0);

  /*Third Goal*/
  //Spit out blue ball
  backwardCoast(450, 127, 1800);
  brake(20);
  autonRollers(127);
  if(left){
    turnLeft(900, 110);
  }
  else{
    turnLeft(900, 110);
  }

  //ejecting ball
  autonFlywheel(-127);

  //pick up corner ball
  forwardCoast(700, 127, 900);
  forwardCoast(1200, 115, 900);
  //stop ejection
  autonFlywheel(0);
  forwardCoast(900, 80, 900);

  brake(-20);
  delay(100);

  //pick up wall ball
  if(left){
    turnLeft(600, 110);
  }
  else{
    turnLeft(600, 110);
  }
  setFlywheel(0, true);

  forwardCoast(500, 100, 600);
  setRollers(0, false);
  forwardCoast(500, 100, 600);
  delay(200);
  backwardCoast(1500, 110, 900);
  brake(20);

  if(left)
  {
    turnLeft(1300, 110);
  }
  else
  {
    turnRight(1300, 110);
  }
  forwardCoast(1200, 110, 1300);
  assignDriveMotorsAuton(10);
  cornerGoal(true, CORNERGOALTIMEOUT);
  assignDriveMotorsAuton(0);
  delay(100);
}

void secondHalf(){
  int headingChange = 1800;

  //spit out one ball
  autonRollers(127);
  setFlywheel(0, false);
  backwardCoast(200, 127, -1325 + headingChange);
  autonRollers(0);
  delay(50);
  autonRollers(-127);
  backwardCoast(1000, 120, -1325 + headingChange);
  autonRollers(0);
  delay(200);
  autonRollers(127);


  if(left){
    turnRight(-900 + headingChange, 110);
  }
  else{
    turnLeft(-900 + headingChange, 110);
  }
  setRollers(0, false);

  forwardCoast(2000, 110, 900);
  delay(300);
  run_flywheel = false; //Second Goal

  //turn to pick up ball
  backwardCoast(400, 110, 900);
  brake(20);
  //pick up other ball
  if(left){
    turnLeft(750 + headingChange, 110);
  }else{
    turnRight(750 + headingChange, 110);
  }
  //eject ball
  autonFlywheel(-127);
  forwardCoast(1500, 120, 750 + headingChange);
  autonFlywheel(0);

  autonRollers(127);
  setFlywheel(0, true);
  forwardCoast(1000, 120, 750 + headingChange);
  setFlywheel(0, true);
  forwardCoast(1100, 100, 750 + headingChange);
  setRollers(0, false);
  brake(-20);

  //shoot second goal
  if(left){
    turnRight(1800 + headingChange, 110);
  }
  else{
    turnLeft(1800 + headingChange, 110);
  }
  //run_flywheel = true;
  forwardCoast(1800, 110, 1800 + headingChange);
  assignDriveMotorsAuton(20);
  middleGoal(true, 3000);
  assignDriveMotorsAuton(0);

  /*Third Goal*/
  //Spit out blue ball
  backwardCoast(500, 120, 1800 + headingChange);
  brake(20);
  autonRollers(127);
  if(left){
    turnLeft(900 + headingChange, 110);
  }
  else{
    turnLeft(900 + headingChange, 110);
  }

  //ejecting ball
  autonFlywheel(-127);

  //pick up corner ball
  forwardCoast(700, 127, 900 + headingChange);
  forwardCoast(1200, 127, 900 + headingChange);
  //stop ejection
  autonFlywheel(0);
  forwardCoast(900, 100, 900 + headingChange);

  brake(-20);
  delay(100);

  //pick up wall ball
  if(left){
    turnLeft(600 + headingChange, 110);
  }
  else{
    turnLeft(600 + headingChange, 110);
  }
  setFlywheel(0, true);

  forwardCoast(500, 120, 600 + headingChange);
  setRollers(0, false);
  forwardCoast(500, 120, 600 + headingChange);
  delay(200);
  backwardCoast(1500, 120, 900 + headingChange);
  brake(20);

  if(left)
  {
    turnLeft(1300 + headingChange, 100);
  }
  else
  {
    turnRight(1300 + headingChange, 100);
  }
  forwardCoast(1250, 120, 1300 + headingChange);
  assignDriveMotorsAuton(20);
  cornerGoal(true, 3000);
  assignDriveMotorsAuton(0);
  delay(100);
}

void progSkills(bool left){
  /*setFlywheel(0, true);
  //First Goal
  //pick up first ball
  setRollers(0, true);
  forwardCoast(1250, 127, 0);
  delay(50);
  brake(-20);

  //align with first goal
  if(left){
    turnLeft(-1275, 110);
  }
  else{
    turnRight(-1275, 110);
  }
  forwardCoast(1400, 127, -1275);

  //shoot first goal
  assignDriveMotorsAuton(10);
  cornerGoal(true, 3000);
  assignDriveMotorsAuton(0);




  firstHalf();

  //go to next ball next to middle goal
  //Fourth Goal
  autonRollers(127);
  setFlywheel(0, false);
  backwardCoast(200, 127, 1300);
  autonRollers(0);
  delay(50);
  autonRollers(-127);
  backwardCoast(1000, 127, 1300);
  brake(20);
  autonRollers(0);
  if(left)
  {
    turnLeft(-200, 100);
  }
  else
  {
    turnRight(-200, 100);
  }

  autonRollers(127);
  //ejecting
  run_flywheel = false;
  autonFlywheel(-127);
  forwardCoast(1000, 127, -200);
  forwardCoast(1000, 115, -200);
  forwardCoast(1200, 75, -200);
  brake(-20);
  autonFlywheel(0);
  run_flywheel = true;

  //turn to shoot 4th goal
  if(left)
  {
    turnLeft(900, 100);
  }
  else
  {
    turnRight(900, 100);
  }*/
  run_flywheel = true;
  setRollers(0, false);
  //run_intake = true;
  forwardCoast(600, 100, 900);
  run_flywheel = false;
  forwardCoast(1200, 100, 900);

  delay(500);
  autonRollers(0);
  assignDriveMotorsAuton(20);
  middleGoal(true, MIDDLEGOALTIMEOUT);
  assignDriveMotorsAuton(0);
  delay(100);

  //Fifth Goal
  backwardCoast(950, 115, 900);
  brake(10);

  autonRollers(127);
  //ejecting blue ball
  autonFlywheel(-127);
  if(left)
  {
    turnLeft(0, 100);
  }
  else
  {
    turnRight(0, 100);
  }
  forwardCoast(2750, 127, 0);
  delay(200);
  setFlywheel(0, false);
  backwardCoast(1000, 127, 0);
  brake(20);

  //turn towards goal
  if(left)
  {
    turnLeft(475, 100);
  }
  else
  {
    turnRight(475, 100);
  }

  forwardCoast(1800, 110, 475);
  autonRollers(0);
  delay(500);
  assignDriveMotorsAuton(10);
  cornerGoalOneRed(true);
  assignDriveMotorsAuton(0);
  delay(100);

  secondHalf();

  autonRollers(127);
  setFlywheel(0, false);
  backwardCoast(200, 127, 3100);
  autonRollers(0);
  delay(50);
  autonRollers(-127);
  backwardCoast(700, 120, 3100);
  brake(20);
  autonRollers(0);
  if(left)
  {
    turnLeft(1800, 110);
  }
  else
  {
    turnRight(1800, 110);
  }

  run_flywheel = false;
  autonRollers(127);
  autonFlywheel(-127);
  forwardCoast(2750, 120, 1800);
  autonFlywheel(0);
  setFlywheel(0, false);
  //setRollers(0, false);
  if(left)
  {
    turnLeft(2700, 110);
  }
  else
  {
    turnRight(2700, 110);
  }
  forwardCoast(500, 110, 2700);

  delay(200);
  autonRollers(0);
  delay(500);
  run_flywheel = false;
  assignDriveMotorsAuton(20);
  middleGoalOneRed(true, 2000);
  assignDriveMotorsAuton(0);

  backwardCoast(400, 120, 2700);
  brake(20);
  autonFlywheel(0);
  delay(200);

  /*Ninth Ball*/
  if(left)
  {
    turnRight(900, 100);
  }
  else
  {
    turnLeft(900, 100);
  }
  autonRollers(127);
  autonFlywheel(-127);
  forwardCoast(800, 127, 900);
  delay(200);
  forwardCoast(400, 110, 900);
  delay(200);
  autonFlywheel(0);
  forwardCoast(1000, 90, 750);

  backwardCoast(800, 70, 750);
  forwardCoast(1000, 90, 750);
  backwardCoast(500, 70, 750);
  forwardCoast(1000, 90, 750);
  backwardCoast(500, 70, 750);

  if(left)
  {
    turnRight(925, 100);
  }
  else
  {
    turnLeft(925, 100);
  }
                      //950
  forwardCoast(300, 70, 925);
  autonFlywheel(127);
  delay(700);
  assignDriveMotorsAuton(-127);
  delay(600);
  assignDriveMotorsAuton(0);
}

void displayInfoAuton(void *param){
   int i = 10;
   lcd_initialize();
   while (true)
   {
      char tempString1[100];
      char tempString2[100];
      char tempString3[100];
      char tempString4[100];
      char tempString5[100];
      char tempString6[100];

      //sprintf(tempString1, "Gyro Value: %d", (int)adi_gyro_get(gyro));
      sprintf(tempString1, "Heading: %d", (int)(currentheading));
      sprintf(tempString2, "IMU Set Value: %d", (int)(imu_get_heading(IMU_PORT)*10));

      sprintf(tempString3, "Battery Voltage: %d", battery_get_voltage());
    //  sprintf(tempString4, "GetHeading Value: %d", getHeading());
      lcd_set_text(1, tempString1);
      lcd_set_text(2, tempString2);
      lcd_set_text(3, tempString3);
      lcd_set_text(4, tempString4);
      lcd_set_text(5, tempString5);
      lcd_set_text(6, tempString6);

      printf("here\n");
      delay(10);
   }
}

void autonomous() {

   imu_sensor = 1;
   autonNumber = 1;
   bool left = true;

   task_t displayInfoTask = task_create(displayInfoAuton, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Display Info Task");
   task_t heading = task_create(getHeading, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Heading Task");
   task_t intakeIndex = task_create(runIntake, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Task");
   task_t flywheelIndex = task_create(runFlywheel, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Task");
   switch (autonNumber)
   {
   case 1:
      progSkills(left);
      break;
   case 2:
      break;
   case 3:
      break;
   case 4:
      break;
   case 5:
      break;
   case 6:
      break;
   default:
      break;
   }
}
