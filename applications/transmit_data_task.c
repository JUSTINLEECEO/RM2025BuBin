#include "transmit_data_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "can.h"
#include "nucCommu.h"

extern RC_Ctl_t remoteCtrl[2];

extern int is_Zimiao;

void transmit_data_task(void const * argument)
{
	uint8_t txmessage[8];
	uint8_t rcmessage[2];
	while (1)
	{
    txmessage[4] = remoteCtrl[0].rc.rockerrx >> 8;
    txmessage[5] = remoteCtrl[0].rc.rockerrx & 0xff;
    txmessage[0] = remoteCtrl[0].rc.rockerlx >> 8;
    txmessage[1] = remoteCtrl[0].rc.rockerlx & 0xff;
    txmessage[2] = remoteCtrl[0].rc.rockerly >> 8;
    txmessage[3] = remoteCtrl[0].rc.rockerly & 0xff;
    txmessage[6] = remoteCtrl[0].rc.switchRight;
    CAN_Tansmit(&hcan1, 0x110, txmessage, 7);
    
		rcmessage[0] = is_Zimiao;
		rcmessage[1] = (uint8_t)get_nuc_control_point()->yaw.data;
		
		CAN_Tansmit(&hcan1, 0x111, rcmessage, 1);
    osDelay(1);
	}
}


