#include "main.h"
#include "motor_task.h"
#include "pid.h"
#include "CAN_receive.h"

// Yaw Gimbal
#define YGVP 40
#define YGVI 5
#define YGVD 0
#define YGVIMAX 10
#define YGVOUTMAX 2000

#define YGSP 40
#define YGSI 5
#define YGSD 0
#define YGSIMAX 10
#define YGSOUTMAX 2000

PID_TypeDef Yaw_Gimbal_V_Pid = {0};
PID_TypeDef Yaw_Gimbal_S_Pid = {0};

void motor_task(void const * argument)
{
	PID_Init(&Yaw_Gimbal_V_Pid, YGVP, YGVI, YGVD, YGVIMAX, YGVOUTMAX, 0.1, 100, 100, 0.02, 0.02, NONE);
	PID_Init(&Yaw_Gimbal_S_Pid, YGSP, YGSI, YGSD, YGSIMAX, YGSOUTMAX, 0.1, 100, 100, 0.02, 0.02, NONE);
	while (1)
	{
		
		CAN_cmd_gimbal(Yaw_Gimbal_V_Pid.Output, 0, 0, 0);
	}
}
