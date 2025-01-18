#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#define MIN_CIRCLE_THRESHOLD 50   // 最低阈值转速（单位：圈数/1000）
#define STALL_TIME_THRESHOLD 200  // 堵转时间阈值（单位：ms）

void shoot_task(void const * argument);

#endif
