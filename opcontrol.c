#include "main.h"
#include "config.h"
#include <stdio.h>
#include <inttypes.h>

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define TOPSENSOR 2475
#define BOTTOMSENSOR 2650

#define TWOBALLMIDDLETIME 1250
#define TWOBALLCORNERTIME 1350
#define ONEBALLCORNERTIME 500
//#define TOPSENSORINDEX 2400
int shootBalls = 0;

int intakeDirection = 0;
int flywheelDirection = 0;
extern adi_gyro_t gyro;

extern void stopAutonTasks();
//extern void displayInfo(void *param);

task_t driveTask;
task_t rolTask;
task_t flyTask;
task_t shootTask;


void assignDriveMotors(int power){
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVELEFTMIDDLE, power);
    motor_move(PORT_DRIVERIGHTFRONT, power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTMIDDLE, power);
    motor_move(PORT_DRIVERIGHTBACK, power);
}

void drive(void* param){
    int i = 100;
    while (true) {
      /*if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y))
      {
        autonomous();
        break;
      }*/
      if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_B)){
        motor_move(PORT_FLYWHEEL, 127);
        assignDriveMotors(-35);
        delay(800);
        assignDriveMotors(60);
        delay(200);
        motor_move(PORT_FLYWHEEL, 0);
        assignDriveMotors(0);
      }
      else{
        int forward = -controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X);
        int turn = controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y);

        motor_move(PORT_DRIVELEFTFRONT, max(-127, min(127, forward + turn)));
        motor_move(PORT_DRIVELEFTMIDDLE, max(-127, min(127, forward + turn)));
        motor_move(PORT_DRIVELEFTBACK, max(-127, min(127, forward + turn)));
        motor_move(PORT_DRIVERIGHTFRONT, -max(-127, min(127, forward - turn)));
        motor_move(PORT_DRIVERIGHTMIDDLE, -max(-127, min(127, forward - turn)));
        motor_move(PORT_DRIVERIGHTBACK, -max(-127, min(127, forward - turn)));
        delay(20);
      }
    }
}
void preGoal(){
  int curtime = millis();
  while(adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP) > TOPSENSOR && (millis() - curtime) < 750)
  {
    motor_move(PORT_FLYWHEEL, 127);
    delay(20);
  }
}
void preGoalMiddle(){
  int curtime = millis();
  while(adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP) > TOPSENSOR && (millis() - curtime) < 1000)
  {
    motor_move(PORT_FLYWHEEL, 127);
    delay(20);
  }
}

void middleGoal(bool auton, int timeout){
  if(!auton)
  {
    preGoal();
  }

  int timeoutTimer = -1;
  if(timeout!= 0)
  {
    timeoutTimer = millis();
  }
  int ballsintake = 0;
  int ballsshoot = 0;
  bool pressed = false;
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  int prevbottom = 0;
  int prevtop = 0;
  int time = 0;
  while(ballsshoot < 2 || ballsintake < 1 || (time != 0 && (millis() - time) < 300)){
    if(timeoutTimer != -1 && millis() - timeoutTimer >= timeout)
    {
      break;
    }
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(!auton && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))){
      motor_move(PORT_FLYWHEEL, 0);
      motor_move(PORT_ROLLERS, 0);
      break;
    }
    if(time != 0){
      if((millis() - time) > 300){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR){
      ballsintake++;
      if(ballsintake >= 1){
        motor_move(PORT_ROLLERS, 0);
      }
    }

    prevbottom = curbottom;
    int curtop = adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP);
    if(curtop > TOPSENSOR && prevtop <= TOPSENSOR){
      ballsshoot++;
      if(ballsshoot >= 2){
        time = millis();
      }
    }
    prevtop = curtop;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}
void middleGoal2and6(bool auton, int timeout){

  int timeoutTimer = -1;
  if(timeout!= 0)
  {
    timeoutTimer = millis();
  }
  int ballsintake = 0;
  bool pressed = false;

  int bottomTimer = millis();
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  int prevbottom = 0;
  int prevtop = 0;
  int time = millis();
  while(ballsintake < 1 || (millis() - time) < TWOBALLMIDDLETIME){
    if(timeoutTimer != -1 && millis() - timeoutTimer >= timeout)
    {
      break;
    }
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(!auton && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))){
      motor_move(PORT_FLYWHEEL, 0);
      motor_move(PORT_ROLLERS, 0);
      break;
    }
    if((millis() - time) > TWOBALLMIDDLETIME){
      motor_move(PORT_FLYWHEEL, 0);
    }
    if(bottomTimer != 0 && (bottomTimer - millis()) < 200)
    {

    }
    else if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR){
      bottomTimer = 0;
      ballsintake++;
      motor_move(PORT_ROLLERS, 0);
    }
    prevbottom = curbottom;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}
void middleGoalOneRed(bool auton, int timeout){
  preGoalMiddle();
  int timeoutTimer = -1;
  if(timeout!= 0)
  {
    timeoutTimer = millis();
  }
  int ballsintake = 0;
  int ballsshoot = 0;
  bool pressed = false;
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  int prevbottom = 0;
  int prevtop = 0;
  int time = 0;
  while(ballsshoot < 1 || ballsintake < 1 || (time != 0 && (millis() - time) < 300)){
    if(timeoutTimer != -1 && millis() - timeoutTimer >= timeout)
    {
      break;
    }
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(time != 0){
      if((millis() - time) > 450){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR){
      ballsintake++;
      if(ballsintake >= 1){
        motor_move(PORT_ROLLERS, 0);
      }
    }

    prevbottom = curbottom;
    int curtop = adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP);
    if(curtop > TOPSENSOR && prevtop <= TOPSENSOR){
      ballsshoot++;
      if(ballsshoot >= 1){
        time = millis();
      }
    }
    prevtop = curtop;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}

void cornerGoalOneRed(bool auton){
  //preGoal();
  int ballsintake = 0;
  int ballsshoot = 0;
  bool pressed = false;
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  int prevbottom = 0;
  int prevtop = 0;
  int timetop = 0;
  int timebot = 0;
  while(ballsshoot < 1 || ballsintake < 2 || (timetop != 0 && (millis() - timetop) < 300)){
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(!auton && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))){
      motor_move(PORT_FLYWHEEL, 0);
      motor_move(PORT_ROLLERS, 0);
      break;
    }
    if(timetop != 0){
      if((millis() - timetop) > 300){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR && ballsintake < 2){
      ballsintake++;
      if(ballsintake >= 2){
        motor_move(PORT_ROLLERS, 0);
      }
    }
    prevbottom = curbottom;

    int curtop = adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP);
    if(curtop > TOPSENSOR && prevtop <= TOPSENSOR){
      ballsshoot++;
      if(ballsshoot >= 1){
        timetop = millis();
      }
    }
    prevtop = curtop;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}
void cornerGoalOneRedFast(bool auton){
  //preGoal();
  int ballsintake = 0;
  int ballsshoot = 0;
  bool pressed = false;
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  int prevbottom = 0;
  int prevtop = 0;
  int timetop = millis();
  int timebot = 0;
  while(ballsintake < 2 || (millis() - timetop) < ONEBALLCORNERTIME){
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(!auton && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))){
      motor_move(PORT_FLYWHEEL, 0);
      motor_move(PORT_ROLLERS, 0);
      break;
    }
    if(timetop != 0){
      if((millis() - timetop) > ONEBALLCORNERTIME){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR && ballsintake < 2){
      ballsintake++;
      if(ballsintake >= 2){
        motor_move(PORT_ROLLERS, 0);
      }
    }
    prevbottom = curbottom;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}

void cornerGoal(bool auton, int timeout){
  preGoal();
  int timeoutTimer = -1;
  if(timeout!= 0)
  {
    timeoutTimer = millis();
  }
  int ballsintake = 0;
  int ballsshoot = 0;
  bool pressed = false;
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  intakeDirection = 1;
  int prevbottom = 0;
  int prevtop = 0;
  int timetop = 0;
  int timebot = 0;
  while(ballsshoot < 2 || ballsintake < 2 || (timetop != 0 && (millis() - timetop) < 300)){
    if(timeoutTimer != -1 && millis() - timeoutTimer >= timeout)
    {
      break;
    }
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(!auton && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))){
      motor_move(PORT_FLYWHEEL, 0);
      motor_move(PORT_ROLLERS, 0);
      intakeDirection = 0;
      break;
    }
    if(timetop != 0){
      if((millis() - timetop) > 300){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(timebot != 0)
    {
      if((millis() - timebot) > 200){
        motor_move(PORT_ROLLERS, -127);
        timebot = 0;
      }
    }
    else if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR && ballsintake < 2){
      ballsintake++;
      if(ballsintake == 1)
      {
        motor_move(PORT_ROLLERS, 0);
        intakeDirection = 0;
        timebot = millis();
      }
      if(ballsintake >= 2){
        motor_move(PORT_ROLLERS, 0);
        intakeDirection = 0;
      }
    }

    prevbottom = curbottom;
    int curtop = adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP);
    if(curtop > TOPSENSOR && prevtop <= TOPSENSOR){
      ballsshoot++;
      if(ballsshoot >= 2){
        timetop = millis();
      }
    }
    intakeDirection = 0;
    prevtop = curtop;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}
void cornerGoalFast(bool auton, int timeout){
  int timeoutTimer = -1;
  if(timeout!= 0)
  {
    timeoutTimer = millis();
  }
  int ballsintake = 0;
  motor_move(PORT_FLYWHEEL, 127);
  motor_move(PORT_ROLLERS, -127);
  intakeDirection = 1;
  int prevbottom = 0;
  int prevtop = 0;
  int timetop = millis();
  int timebot = 0;
  while(ballsintake < 2 || (millis() - timetop) < TWOBALLCORNERTIME){
    if(timeoutTimer != -1 && millis() - timeoutTimer >= timeout)
    {
      break;
    }
    int curbottom = adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM);
    if(!auton && (controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN))){
      motor_move(PORT_FLYWHEEL, 0);
      motor_move(PORT_ROLLERS, 0);
      intakeDirection = 0;
      break;
    }
    if(timetop != 0){
      if((millis() - timetop) > TWOBALLCORNERTIME){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(timebot != 0)
    {
      if((millis() - timebot) > 200){
        motor_move(PORT_ROLLERS, -127);
        timebot = 0;
      }
    }
    else if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR && ballsintake < 2){
      ballsintake++;
      if(ballsintake == 1)
      {
        motor_move(PORT_ROLLERS, 0);
        intakeDirection = 0;
        timebot = millis();
      }
      if(ballsintake >= 2){
        motor_move(PORT_ROLLERS, 0);
        intakeDirection = 0;
      }
    }
    prevbottom = curbottom;

    intakeDirection = 0;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}
void shooting(void* param){
  while(true){
    if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)){
      //middleGoal(false, 0);
      middleGoal2and6(false, 0);
    }
    if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_UP) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_UP)){
      cornerGoalFast(false, 0);
      /*motor_move(PORT_ROLLERS, 127);
      intakeDirection = -1;
      //assignDriveMotors(-127, -127);
      delay(50);*/
    }
    if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_RIGHT) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_RIGHT)){
      motor_move(PORT_ROLLERS, 0);
      motor_move(PORT_FLYWHEEL, 0);
      cornerGoalOneRed(false);
    }
    if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN)){
      motor_move(PORT_ROLLERS, 0);
      motor_move(PORT_FLYWHEEL, 0);
    }
    delay(50);
  }
  motor_move(PORT_ROLLERS, 80);
  delay(500);
  motor_move(PORT_ROLLERS, 0);
}


void rollers(void* param){
    int ballCount = 0;
    while (true) {

        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_A) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_A)){
          bool pressed = false;
          intakeDirection = 1;
          motor_move(PORT_ROLLERS, -127);
          while(adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM) > BOTTOMSENSOR)
          {
            if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN)){
              pressed = true;
              motor_move(PORT_ROLLERS, 0);
              motor_move(PORT_FLYWHEEL, 0);
              intakeDirection = 0;
              break;
            }
            delay(20);
          }
          if(!pressed)
          {
            delay(50);
            motor_move(PORT_ROLLERS, 0);
            intakeDirection = 0;
          }

        }

       if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN)){
              motor_move(PORT_ROLLERS, 0);
              motor_move(PORT_FLYWHEEL, 0);
              intakeDirection = 0;
              flywheelDirection = 0;
       }

        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_R1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_R1)){
          if(intakeDirection == 0 || intakeDirection == -1){
            motor_move(PORT_ROLLERS, -127);
            intakeDirection = 1;
          }
          else{
            motor_move(PORT_ROLLERS, 0);
            intakeDirection = 0;
          }
          delay(200);
        }

        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_R2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_R2)){
          if(intakeDirection == 0 || intakeDirection == 1){
            motor_move(PORT_ROLLERS, 127);
            intakeDirection = -1;
          }
          else{
            motor_move(PORT_ROLLERS, 0);
            intakeDirection = 0;
          }
          delay(200);
        }
        delay(20);
    }
}
void flywheel(void* param){
    flywheelDirection = 0;
    bool pressed = false;
    while(true)
    {
      if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y)|| controller_get_digital(CONTROLLER_PARTNER, DIGITAL_Y)){
        motor_move(PORT_FLYWHEEL, 80);
        delay(3000);
        motor_move(PORT_FLYWHEEL, 0);
      }
      if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_X)){
        while(adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP) > TOPSENSOR)
        {
          flywheelDirection = 1;
          motor_move(PORT_FLYWHEEL, 127);
          delay(20);
          if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN)){
            pressed = true;
            motor_move(PORT_ROLLERS, 0);
            motor_move(PORT_FLYWHEEL, 0);
            flywheelDirection = 0;
            break;
          }
        }
        motor_move(PORT_FLYWHEEL, 0);
        flywheelDirection = 0;
      }
      if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_L1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L1)){
        if(flywheelDirection == 0 || flywheelDirection == -1){
          motor_move(PORT_FLYWHEEL, 127);
          motor_move(PORT_TOPBACK, 127);
          flywheelDirection = 1;
        }
        else{
          motor_move(PORT_FLYWHEEL, 0);
          motor_move(PORT_TOPBACK, 0);
          flywheelDirection = 0;
        }
        delay(200);
      }
      if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_L2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L2)){
        if(flywheelDirection == 0 || flywheelDirection == 1){
          motor_move(PORT_FLYWHEEL, -127);
          motor_move(PORT_TOPBACK, -127);
          flywheelDirection = -1;
        }
        else{
          motor_move(PORT_FLYWHEEL, 0);
          motor_move(PORT_TOPBACK, 0);
          flywheelDirection = 0;
        }
        delay(200);
      }
      delay(20);
    }
}

bool opControl_started=false;

void opcontrol() {
    stopAutonTasks();

    //task_t displayTask = task_create(displayInfo, "PROS", TASK_PRIORITY_DEFAULT,
    //                                 TASK_STACK_DEPTH_DEFAULT, "Display Info Task");
    if(!opControl_started)
    {
      task_t driveTask = task_create(drive, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive Task");
      task_t rolTask = task_create(rollers, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Rol Task");
      task_t flyTask = task_create(flywheel, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Fly Task");
      task_t shootTask = task_create(shooting, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Shooting Task");
      opControl_started  = true;
    }


}
