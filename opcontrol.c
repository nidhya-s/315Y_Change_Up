#include "main.h"
#include "config.h"
#include <stdio.h>
#include <inttypes.h>
extern void initializeDriveMotors();
 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define TOPSENSOR 2400
#define BOTTOMSENSOR 2700
//#define TOPSENSORINDEX 2400
int shootBalls = 0;
extern adi_gyro_t gyro;

/*void assignDriveMotors(int leftSide, int rightSide){
    motor_move(PORT_DRIVELEFTFRONT, leftSide);
    motor_move(PORT_DRIVELEFTBACK, leftSide);
    motor_move(PORT_DRIVERIGHTFRONT, rightSide);
    motor_move(PORT_DRIVERIGHTBACK, rightSide);
}*/

void drive(void* param){
    int i = 100;
    while (true) {
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
void preGoal(){
  int curtime = millis();
  while(adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP) > TOPSENSOR && (millis() - curtime) < 750)
  {
    motor_move(PORT_FLYWHEEL, 127);
    delay(20);
  }
}
void middleGoal(bool auton, int timeout){
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
void middleGoalOneRed(bool auton, int timeout){
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
  preGoal();
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
      break;
    }
    if(timetop != 0){
      if((millis() - timetop) > 300){
        motor_move(PORT_FLYWHEEL, 0);
      }
    }
    if(timebot != 0)
    {
      if((millis() - timebot) > 300){
        motor_move(PORT_ROLLERS, -127);
        timebot = 0;
      }
    }
    else if(curbottom <= BOTTOMSENSOR && prevbottom > BOTTOMSENSOR && ballsintake < 2){
      ballsintake++;
      if(ballsintake == 1)
      {
        motor_move(PORT_ROLLERS, 0);
        timebot = millis();
      }
      if(ballsintake >= 2){
        motor_move(PORT_ROLLERS, 0);
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
    prevtop = curtop;
    delay(50);
  }
  motor_move(PORT_FLYWHEEL, 0);
  motor_move(PORT_ROLLERS, 0);
}
void shooting(void* param){
  while(true){
    if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)){
      middleGoal(false, 0);
    }
    if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_UP) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_UP)){
      cornerGoal(false, 0);
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
    int intakeDirection = 0;
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
    int flywheelDirection = 0;
    bool pressed = false;
    while(true)
    {
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
      if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_B)){
        motor_move(PORT_FLYWHEEL, 127);
        flywheelDirection = 1;
        delay(210);
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
void displayInfo(void *param)
{
   lcd_initialize();
   while (true)
   {
      char tempString1[100];
      char tempString2[100];
      char tempString3[100];
      char tempString4[100];
      char tempString5[100];
      char tempString6[100];

      sprintf(tempString1, "Gyro Value: %d", (int)adi_gyro_get(gyro));
      sprintf(tempString2, "IMU Value: %d", (int)imu_get_heading(IMU_PORT)*10);
      sprintf(tempString3, "Balls Shot: %d", shootBalls);
      sprintf(tempString4, "Line Sensor Value Top: %d", adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP));
      sprintf(tempString5, "Line Sensor Value Bottom: %d", adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM));

      lcd_set_text(1, tempString1);
      lcd_set_text(2, tempString2);
      lcd_set_text(3, tempString3);
      lcd_set_text(4, tempString4);
      lcd_set_text(5, tempString5);
      lcd_set_text(6, tempString6);

      printf("here\n");

      //controller_print(CONTROLLER_MASTER, 0, 0, "RPM: %.2f", motor_get_actual_velocity(PORT_FLYWHEEL));

      delay(10);
   }
}
void opcontrol() {
    task_t driveTask = task_create(drive, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive Task");
    task_t rolTask = task_create(rollers, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Rol Task");
    task_t flyTask = task_create(flywheel, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Fly Task");
    task_t displayInfoTask = task_create(displayInfo, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Display Info Task");
    task_t shootTask = task_create(shooting, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Shooting Task");
}
