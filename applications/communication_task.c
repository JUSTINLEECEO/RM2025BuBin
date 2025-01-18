#include "communication_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "nucCommu.h"
#include "INS_task.h"
#include "usart.h"
#include "bsp_usart.h"
#include "CAN_receive.h"
#include "cmsis_os.h"

static uint8_t GimbalAngleMsg[40];

void communication_task(void const * argument)
{
		int count = 0;
    while (1)
    {
				if(count % 5  ==  0)
				{
						Encode1(GimbalAngleMsg,get_INS()->Yaw,get_INS()->Pitch,get_INS()->Roll);
						HAL_UART_Transmit(&huart1, GimbalAngleMsg, 17, 100);
						usart1_tx_dma_enable(GimbalAngleMsg, 17);
				}
				if(count % 100 == 0)
				{
//					Encode2(GimbalAngleMsg,get_refree_point()->robot_id>100?1:0,get_refree_point()->robot_id>100?get_refree_point()->robot_id-100:get_refree_point()->robot_id,1,robotIsAuto());
						Encode2(GimbalAngleMsg, COLOR_RED, STANDARD_3, Normal, Attack_free);
						HAL_UART_Transmit(&huart1, GimbalAngleMsg, 9, 100);
						usart1_tx_dma_enable(GimbalAngleMsg, 9);
				}
				count++;
				if(count>1000)
					count = 0;
				osDelay(1);
		}
}
