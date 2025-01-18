#include "main.h"
#include "motor_task.h"
#include "pid.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "usart.h"
#include "pid.h"
#include "nucCommu.h"

#define MIN_CIRCLE_THRESHOLD 40 
#define STALL_TIME_THRESHOLD 100 

extern int is_Zimiao;

extern motor_measure_t motor_chassis[7];
extern PID_TypeDef Motor_VPID[4];
extern PID_TypeDef Motor_SPID;
extern PID_TypeDef Gimbal_VPID;
extern PID_TypeDef Gimbal_SPID;
extern char rxmessage[BUFFERSIZE];
long int chassis1TargetCircle = 0;
extern uint8_t status;
extern int gimbal1RawCircle, gimbalpitchRawCircle;
long int gimbal1TargetCircle = 0;

int gimbal1RawAngle, gimbalpitchRawAngle;
long int gimbal1Circle, gimbalpitchCircle;

int gimbal1TargetSpeed = 0, gimbalpitchTargetSpeed = 0;

extern CAN_RxHeaderTypeDef cboard_header;
extern uint8_t cboard_data[8];
extern uint8_t KeyState;

extern RC_Ctl_t remoteCtrl[2];

void motor_task(void *argument)
{
  int i = 0;
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
  uint8_t STOPFLAG = 0, Stopflag = 0;
  const TickType_t xFrequency = pdMS_TO_TICKS(5);
  while (get_yaw_gimbal_motor_measure_point()->ecd == 0 || get_pitch_gimbal_motor_measure_point()->ecd == 0)
  {
    osDelay(10);
  }
	
  if (get_yaw_gimbal_motor_measure_point()->ecd >= 150)
  {
    gimbal1RawCircle--;
  }

  osDelay(5);

  for (;;)
  {
    xLastWakeTime = xTaskGetTickCount();
    gimbalpitchRawAngle = get_pitch_gimbal_motor_measure_point()->ecd / 8.1920f;
    gimbalpitchCircle = gimbalpitchRawCircle * 1000 + gimbalpitchRawAngle;

    gimbalpitchTargetAngleR[0] += remoteCtrl[0].rc.rockerry / 400;
    if (gimbalpitchTargetAngleR[0] < -100)
      gimbalpitchTargetAngleR[0] = -100;
    else if (gimbalpitchTargetAngleR[0] > 14)
      gimbalpitchTargetAngleR[0] = 14;

    gimbalpitchTargetAngle = gimbalpitchTargetAngleR[0];
    gimbalpitchSpeed = get_pitch_gimbal_motor_measure_point()->speed_rpm;
		
		// ------------------×ÔÃé
		if (is_Zimiao)
		{
			gimbalpitchTargetAngle += (get_nuc_control_point()->pitch.data);
		}

    // if (get_pitch_gimbal_motor_measure_point()->ecd >= 140 && get_pitch_gimbal_motor_measure_point()->ecd <= 7340)
    //   CAN_cmd_gimbal(0, 0, 0, 0);
    // else
    {
      gimbalpitchTargetSpeed = PID_Calculate(&Gimbal_SPID, gimbalpitchCircle, gimbalpitchTargetAngle);
      PID_Calculate(&Gimbal_VPID, gimbalpitchSpeed, gimbalpitchTargetSpeed);
      CAN_cmd_gimbal(0, (int16_t)Gimbal_VPID.Output, 0, 0);
    }

    // sprintf(str, "%d,%d,%d\n",
    //         (int16_t)(((int16_t)txmessage[0]) << 8 | txmessage[1]), remoteCtrl[0].rc.rockerlx, (int)gimbal1TargetAngle);
    // HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 0x08);

    // // gimbal1Speed = get_yaw_gimbal_motor_measure_point()->speed_rpm;

    // // // classic1TargetSpeed = (int16_t)((int16_t)cboard_data[0] << 8 | (int16_t)cboard_data[1]);
    // // // classic1Speed = (int16_t)((int16_t)cboard_data[2] << 8 | (int16_t)cboard_data[3]);
    // // // chassis1Circle = (int32_t)((int32_t)cboard_data[4] << 24 | (int32_t)cboard_data[5] << 16 |
    // // //                            (int32_t)cboard_data[6] << 8 | (int32_t)cboard_data[7]);
    // // // sprintf(str, "%d,%d,%d,%d,%d\n", gimbal1TargetSpeed, gimbal1Speed,
    // // //         classic1TargetSpeed, classic1Speed, chassis1Circle);
    // // // HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 0x04);

    // gimbal1RawAngle = get_pitch_gimbal_motor_measure_point()->ecd / 8.1920f;
    // gimbal1Circle = gimbal1RawCircle * 1000 + gimbal1RawAngle;
    // gimbal1TargetCircle += p;

    // if (remoteCtrl[0].rc.switchLeft == 1)
    // {
    //   gimbal1TargetAngleR[0] += remoteCtrl[0].rc.rockerlx / 100;
    //   gimbal1TargetAngle = gimbal1TargetAngleR[0];
    // }
    // else if (remoteCtrl[0].rc.switchLeft == 2)
    // {
    //   gimbal1TargetAngleR[1] += remoteCtrl[0].rc.rockerlx / 100;
    //   gimbal1TargetAngle = gimbal1TargetAngleR[1];
    // }
    // else if (remoteCtrl[0].rc.switchLeft == 3)
    // {
    //   gimbal1TargetAngleR[2] += remoteCtrl[0].rc.rockerlx / 100;
    //   gimbal1TargetAngle = gimbal1TargetAngleR[2];
    // }

    // gimbal1TargetSpeed = PID_Calculate(&Gimbal_SPID, gimbal1Circle, gimbal1TargetAngle);
    // // gimbal1TargetSpeed = -10;
    // gimbal1TargetCurrent = PID_Calculate(&Gimbal_VPID, gimbal1Speed, gimbal1TargetSpeed);
    // CAN_cmd_gimbal(gimbal1TargetCurrent, 0, 0, 0);

    // gimbal1TargetSpeed = PID_Calculate(&Gimbal_SPID, gimbal1Circle, gimbal1TargetAngle);
    // gimbal1TargetCurrent = PID_Calculate(&Gimbal_VPID, gimbal1Speed, gimbal1TargetSpeed);
    // CAN_cmd_gimbal(gimbal1TargetCurrent, 0, 0, 0);

    // sprintf(str, "%d,%d,%d\n", get_yaw_gimbal_motor_measure_point()->ecd, get_pitch_gimbal_motor_measure_point()->ecd, get_trigger_motor_measure_point()->ecd);
    // HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 0x08);

    // if (KeyState == 1)
    // {
    //   flag = 1;
    //   p = -p;
    //   KeyState = 0;
    // }
    // txdata[0] = (uint8_t)(remoteCtrl[0].rc.rockerrx >> 8);
    // txdata[1] = (uint8_t)remoteCtrl[0].rc.rockerrx;
    // txdata[2] = (uint8_t)remoteCtrl[0].rc.switchLeft;
    // txdata[3] = (uint8_t)remoteCtrl[0].rc.switchRight;
    // CAN_Tansmit(&hcan2, 0x215, txdata, 4);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // osDelay(4);
  }
  /* USER CODE END MotorCtrl */
}


