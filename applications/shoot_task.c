#include "main.h"
#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "pid.h"
#include "CAN_receive.h"
#include "nucCommu.h"

extern int is_Zimiao;

extern RC_Ctl_t remoteCtrl[2];
extern PID_TypeDef Motor_VPID[4];
extern PID_TypeDef Motor_SPID;
extern PID_TypeDef Gimbal_VPID;
extern PID_TypeDef Gimbal_SPID;

int shootRawAngle;
extern int shootRawCircle;

void shoot_task(const void *argument)
{
  char str[128];
  int16_t gimbal1Speed, gimbalpitchSpeed;
  float gimbal1TargetAngleR[3] = {0, 500, 300};
  float gimbalpitchTargetAngleR[3] = {-50};
  float gimbal1TargetAngle, gimbalpitchTargetAngle;
  int32_t chassis1Circle, chassis1LastCircle, shootLastCircle, classic1TargetSpeed, classic1Speed;
  int32_t shootCircle, shootSpeed;
  static int16_t gimbal1TargetCurrent;
  TickType_t xLastWakeTime;
  int16_t chassis1Speed, chassis2Speed, chassis1TargetSpeed, chassis2TargetSpeed, shootTargetSpeed;
  static int flag = 0, cmd = 400, p = 40;
  uint8_t txdata[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  uint8_t STOPFLAG = 0, Stopticks = 0; // 堵转标志，堵转计数器
  int backtimes = 15, i = 0;
  osDelay(2000);
  const TickType_t xFrequency = pdMS_TO_TICKS(5);
	
  for (;;)
  {
    xLastWakeTime = xTaskGetTickCount();
    chassis1Speed = get_chassis_motor_measure_point(0)->speed_rpm;
    chassis2Speed = get_chassis_motor_measure_point(1)->speed_rpm;
    if (remoteCtrl[0].rc.switchLeft == 1)
    {
      chassis1TargetSpeed = 4000;
      chassis2TargetSpeed = -4000;
      shootTargetSpeed = 2000;
    }
    else
    {
      chassis1TargetSpeed = 0;
      chassis2TargetSpeed = 0;
      shootTargetSpeed = 0;
    }
		
		// ------------------自瞄
		if (is_Zimiao)
		{
			if (!get_nuc_control_point()->is_fire)
			{
				chassis1TargetSpeed = 0;
				chassis2TargetSpeed = 0;
				shootTargetSpeed = 0;
			}
		}
		
    PID_Calculate(&Motor_VPID[0], chassis1Speed, chassis1TargetSpeed);
    PID_Calculate(&Motor_VPID[1], chassis2Speed, chassis2TargetSpeed);
    CAN_cmd_chassis(Motor_VPID[0].Output, Motor_VPID[1].Output, 0, 0);

    // shoot
    // shootTargetSpeed = 500;
    shootRawAngle = get_trigger_motor_measure_point()->ecd / 8.1920f;
    shootLastCircle = shootCircle;
    shootCircle = shootRawCircle * 1000 + shootRawAngle;
    if (STOPFLAG == 1)
    {
      // 堵转后倒转200ms
      // while (1)
      // {
      //   osDelay(114514);
      // }
      for (int i = 0; i < 5; i++)
      {
        // CAN_cmd_shoot(-2000);
        osDelay(10);
      }

      // 设置标志位，接下来500ms内不再检测堵转
      STOPFLAG = 2;
      Stopticks = 100;  // 500ms
    }
    else if (STOPFLAG == 2)
    {
      // 等待500ms
      if (Stopticks > 0)
      {
        Stopticks--;
      }
      else
      {
        // 500ms后重置标志位
        STOPFLAG = 0;
        Stopticks = 0;
      }
    }
    else
    {
      if (Stopticks > STALL_TIME_THRESHOLD / 5)
      {
        STOPFLAG = 1;
        Stopticks = 0;
      }
      else
      {
        if (shootCircle - shootLastCircle < MIN_CIRCLE_THRESHOLD && shootTargetSpeed > 100)
        {
          Stopticks++;
        }
        else
        {
          Stopticks = 0;
        }
      }
      PID_Calculate(&Motor_VPID[2], get_trigger_motor_measure_point()->speed_rpm, shootTargetSpeed);
      CAN_cmd_shoot(Motor_VPID[2].Output);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}
