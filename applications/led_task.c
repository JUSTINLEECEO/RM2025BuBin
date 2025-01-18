#include "main.h"
#include "led_task.h"
#include "cmsis_os.h"
#include "remote_control.h"

extern RC_Ctl_t remoteCtrl[2];
extern RC_ctrl_t rc_ctrl;

void led_task(void const * argument)
{
	uint8_t status = 0;
  while (1)
  {
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, (status >> 0) & 0x01);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (status >> 1) & 0x01);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, (status >> 2) & 0x01);
		
    status++;
    osDelay(500);
		
//		if (remoteCtrl[0].rc.switchLeft == 0)
//		{
//			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
//		}
//		else if (remoteCtrl[0].rc.switchLeft == 1)
//		{
//			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
//		}
//		else 
//		{
//			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
//		}
		
//		if (rc_ctrl.rc.s[0] == 0)
//		{
//			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
//		}
//		else if (rc_ctrl.rc.s[0] == 1)
//		{
//			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
//		}
//		else 
//		{
//			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
//		}
  }
}
