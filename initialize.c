#include "main.h"
#include "config.h"
#include "pros/imu.h"

extern task_t headingTask;
extern task_t intakeIndexTask;
extern task_t flywheelIndexTask;

extern task_t driveTask;
extern task_t rolTask;
extern task_t flyTask;
extern task_t shootTask;

extern bool run_flywheel;
extern bool run_intake;

int autonNumber = 3;
bool left = false;
int32_t a;
adi_gyro_t gyro;



#define NUMAUTONS 3

static const char *btnm_map[] = {"PROG", "RED", "BLUE", ""};
static const char *auton_strings[] = {"PROG", "RED", "BLUE", ""};

static lv_res_t btnm_action(lv_obj_t *btnm, const char *txt)
{
   for (int i = 0; i < sizeof(auton_strings) / sizeof(auton_strings[0]); i++)
   {
      printf("%s\n", auton_strings[i]);
      printf("%s\n", txt);
      printf("-----------\n");
      if (strcmp(auton_strings[i], txt) == 0)
      {
         autonNumber = i + 1;
         break;
      }
      lv_btnm_set_toggle(btnm, true, autonNumber);
   }

   return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

static lv_res_t btnm_action_color(lv_obj_t *btnm, const char *txt)
{
   left = false;
   lv_btnm_set_toggle(btnm, true, 1);
   lv_btnm_set_toggle(btnm, true, 2);
   printf("FUNCTION CALLED");
   if (strcmp(txt, "Red") == 0)
   {
      left = true;
   }
   else if (strcmp(txt, "Blue") == 1)
   {
      left = false;
   }

   return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}
void initializeDriveMotors(){
    motor_set_gearing(PORT_DRIVELEFTFRONT, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVERIGHTFRONT, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVELEFTBACK, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVERIGHTBACK, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVELEFTMIDDLE, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVERIGHTMIDDLE, E_MOTOR_GEARSET_18);
    motor_set_reversed(PORT_DRIVELEFTFRONT, true);
    motor_set_reversed(PORT_DRIVERIGHTFRONT, true);
    motor_set_reversed(PORT_DRIVELEFTBACK, true);
    motor_set_reversed(PORT_DRIVERIGHTBACK, true);
    motor_set_reversed(PORT_DRIVELEFTMIDDLE, true);
    motor_set_reversed(PORT_DRIVERIGHTMIDDLE, true);
}

/*void displayInfo(void *param){
   lcd_initialize();
   while (true)
   {
      char tempString1[100];
      char tempString2[100];
      char tempString3[100];

      sprintf(tempString1, "IMU Value: %d", (int)imu_get_heading(IMU_PORT)*10);
      sprintf(tempString2, "Line Sensor Value Top: %d", adi_analog_read_calibrated(LINE_TRACKER_BALL_TOP));
      sprintf(tempString3, "Line Sensor Value Bottom: %d", adi_analog_read_calibrated(LINE_TRACKER_BALL_BOTTOM));

      lcd_set_text(1, tempString1);
      lcd_set_text(2, tempString2);
      lcd_set_text(3, tempString3);

      delay(25);
   }
}*/

void stopAllMotors()
{
  motor_move(PORT_DRIVELEFTFRONT, 0);
  motor_move(PORT_DRIVELEFTMIDDLE, 0);
  motor_move(PORT_DRIVELEFTBACK, 0);
  motor_move(PORT_DRIVERIGHTFRONT, 0);
  motor_move(PORT_DRIVERIGHTMIDDLE, 0);
  motor_move(PORT_DRIVERIGHTBACK, 0);
  motor_move(PORT_ROLLERS, 0);
  motor_move(PORT_FLYWHEEL, 0);
}


void stopAutonTasks(){
  run_flywheel = false;;
  run_intake = false;
  //task_delete(headingTask);
  //task_delete(intakeIndexTask);
  //task_delete(flywheelIndexTask);
  stopAllMotors();
}

void stopDriverTasks(){
 // task_delete(driveTask);
 // task_delete(rolTask);
 // task_delete(flyTask);
 // task_delete(shootTask);
 // stopAllMotors();
}

void disabled(){
  run_flywheel = false;;
  run_intake = false;
//  stopAutonTasks();
//  stopDriverTasks();
//  delay(100);
}


void  reset_imu() {
   imu_reset(IMU_PORT);
   int time = millis();
   int iter = 0;
   while (imu_get_status(IMU_PORT) == EAGAIN) {
    iter += 10;
    delay(10);
   }
}


void initialize(){
    motor_set_reversed(PORT_DRIVERIGHTFRONT, true);
    motor_set_reversed(PORT_DRIVERIGHTBACK, true);
    motor_set_reversed(PORT_DRIVERIGHTMIDDLE, true);
    reset_imu();

    //lcd_initialize();
   //these lines go in initialize() in initialize
      adi_port_set_config(LINE_TRACKER_LEFT,ADI_ANALOG_IN);
      adi_port_set_config(LINE_TRACKER_MIDDLE,ADI_ANALOG_IN);
      adi_port_set_config(LINE_TRACKER_RIGHT,ADI_ANALOG_IN);
      adi_port_set_config(LINE_TRACKER_BALL_TOP,ADI_ANALOG_IN);
      adi_port_set_config(LINE_TRACKER_BALL_BOTTOM,ADI_ANALOG_IN);
      adi_analog_calibrate(LINE_TRACKER_LEFT);
      adi_analog_calibrate(LINE_TRACKER_MIDDLE);
      adi_analog_calibrate(LINE_TRACKER_RIGHT);
      adi_analog_calibrate(LINE_TRACKER_BALL_TOP);
      adi_analog_calibrate(LINE_TRACKER_BALL_BOTTOM);

}


void competition_initialize()
{
   lv_theme_alien_init(40, NULL);

   lv_obj_t *title = lv_label_create(lv_scr_act(), NULL);
   lv_label_set_text(title, "Auton Selection");
   lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

   lv_obj_t *btnm = lv_btnm_create(lv_scr_act(), NULL);
   lv_btnm_set_map(btnm, btnm_map);
   lv_btnm_set_action(btnm, btnm_action);
   lv_obj_set_size(btnm, LV_HOR_RES - 40, LV_VER_RES / 3);
   lv_obj_align(btnm, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

   /*lv_obj_t *allianceM = lv_btnm_create(lv_scr_act(), NULL);
   lv_btnm_set_map(allianceM, alliance_map);
   lv_btnm_set_action(allianceM, btnm_action_color);
   lv_obj_set_size(allianceM, LV_HOR_RES - 40, 50);
   lv_obj_align(allianceM, btnm, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);*/
}
