/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INS_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId motor_taskHandle;
osThreadId communication_taskHandle;
osThreadId INS_taskHandle;
osThreadId led_taskHandle;
osThreadId transmit_data_taskHandle;
osThreadId transmit_data_taskHandle;
osThreadId shoot_taskHandle;
osThreadId rc_taskHandle;
/* USER CODE END Variables */
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void motor_task(void const * argument);
extern void communication_task(void const * argument);
extern void led_task(void const * argument);
extern void transmit_data_task(void const * argument);
void start_INS_task(void const * argument);
extern void shoot_task(void const * argument);
extern void rc_task(void const * argument);
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(motor, motor_task, osPriorityHigh, 0, 512);
	motor_taskHandle = osThreadCreate(osThread(motor), NULL);
	
	osThreadDef(communication, communication_task, osPriorityHigh, 0, 512);
	communication_taskHandle = osThreadCreate(osThread(communication), NULL);
	
	osThreadDef(INS, start_INS_task, osPriorityNormal, 0, 512);
	INS_taskHandle = osThreadCreate(osThread(INS), NULL);
	
//	osThreadDef(led, led_task, osPriorityLow, 0, 128);
//	led_taskHandle = osThreadCreate(osThread(led), NULL);
	
	osThreadDef(transmit_data, transmit_data_task, osPriorityNormal, 0, 512);
	transmit_data_taskHandle = osThreadCreate(osThread(transmit_data), NULL);
	
	osThreadDef(shoot, shoot_task, osPriorityHigh, 0, 512);
	shoot_taskHandle = osThreadCreate(osThread(shoot), NULL);

//	osThreadDef(rc, rc_task, osPriorityAboveNormal, 0, 128);
//	rc_taskHandle = osThreadCreate(osThread(rc), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {;
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void start_INS_task(void const * argurment)
{
	INS_Init();
	while(1){
		INS_task();
		osDelay(1);
	}
}
/* USER CODE END Application */
