#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "usart.h"

extern RC_Ctl_t remoteCtrl[2];
extern uint8_t remoteMessage[36];

void rc_task(void const * argument)
{
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  remoteReceive(remoteMessage);
	osDelay(1);
}
