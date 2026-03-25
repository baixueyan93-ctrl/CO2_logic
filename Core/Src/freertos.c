/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "task_led.h"    // 引入 LED 业务
//#include "task_buzzer.h" // 引入 蜂鸣器 业务
#include "task_panel.h"  // 引入 面板 业务
#include "task_rs485_log.h"// 引入 通信 业务
//#include "task_XKC_Y20_V.h"  // 引入 XKC_Y20_V 液位计业务
#include "task_adc.h"    // 引入 ADC 业务
#include "task_sht30.h"   // SHT30 温湿度采集
#include "task_exv.h"     // 电子膨胀阀测试
#include "bsp_i2c_mutex.h" // I2C1 总线互斥锁
#include "sys_state.h"    // 引入系统状态头文件
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

/* USER CODE END Variables */
osThreadId Task_LEDHandle;
osThreadId Task_RS485Handle;
osThreadId TaskPanelHandle;
osThreadId Task_ADCHandle;
osThreadId Task_SHT30Handle;
osThreadId Task_EXVHandle;
osMutexId EEPROM_MutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_LED(void const * argument);
void StartTask_RS485(void const * argument);
void StartTask03(void const * argument);
void StartTask_ADC(void const * argument);
void StartTask_SHT30(void const * argument);
void StartTask_EXV(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of EEPROM_Mutex */
  osMutexDef(EEPROM_Mutex);
  EEPROM_MutexHandle = osMutexCreate(osMutex(EEPROM_Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  BSP_I2C1_MutexInit();  // 最优先初始化硬件互斥锁
  SysState_Init();       // 必须在调用任务前初始化全局变量值和系统量
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
  /* definition and creation of Task_LED */
  osThreadDef(Task_LED, StartTask_LED, osPriorityNormal, 0, 128);
  Task_LEDHandle = osThreadCreate(osThread(Task_LED), NULL);

  /* definition and creation of Task_RS485 */
  osThreadDef(Task_RS485, StartTask_RS485, osPriorityHigh, 0, 1024);
  Task_RS485Handle = osThreadCreate(osThread(Task_RS485), NULL);

  /* definition and creation of TaskPanel */
  osThreadDef(TaskPanel, StartTask03, osPriorityBelowNormal, 0, 512);
  TaskPanelHandle = osThreadCreate(osThread(TaskPanel), NULL);

  /* definition and creation of Task_ADC */
  osThreadDef(Task_ADC, StartTask_ADC, osPriorityNormal, 0, 512);
  Task_ADCHandle = osThreadCreate(osThread(Task_ADC), NULL);

  /* definition and creation of Task_SHT30 */
  osThreadDef(Task_SHT30, StartTask_SHT30, osPriorityBelowNormal, 0, 256);
  Task_SHT30Handle = osThreadCreate(osThread(Task_SHT30), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 电子膨胀阀测试任务 (普通优先级, 256字栈) */
  osThreadDef(Task_EXV, StartTask_EXV, osPriorityNormal, 0, 256);
  Task_EXVHandle = osThreadCreate(osThread(Task_EXV), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask_LED */
/**
  * @brief  Function implementing the Task_LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_LED */
void StartTask_LED(void const * argument)
{
  /* USER CODE BEGIN StartTask_LED */
  /* Infinite loop */
Task_LED_Process(argument); // 直接调用外部文件封装的代码
  for(;;) { osDelay(1); }     // 安全保底
  /* USER CODE END StartTask_LED */
}

/* USER CODE BEGIN Header_StartTask_RS485 */
/**
* @brief Function implementing the Task_RS485 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_RS485 */
void StartTask_RS485(void const * argument)
{
  /* USER CODE BEGIN StartTask_RS485 */
  /* Infinite loop */
Task_RS485Log_Process(argument); // 调用函数业务
  for(;;) { osDelay(1); }          // 安全保底
  /* USER CODE END StartTask_RS485 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the TaskPanel thread.
* @param argument: Not used
* @retval None
*/
extern void Task_Panel_Process(void const *argument); // 外部声明
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
Task_Panel_Process(argument); // 面板任务写数据到屏幕和系统量
  for(;;) { osDelay(1); }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask_ADC */
/**
* @brief Function implementing the Task_ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ADC */
void StartTask_ADC(void const * argument)
{
  /* USER CODE BEGIN StartTask_ADC */
	// 注意顺序: 先让函数内写好的 ADC 更新大循环跑
  Task_ADC_Process(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_ADC */
}

/* USER CODE BEGIN Header_StartTask_SHT30 */
/**
* @brief Function implementing the Task_SHT30 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_SHT30 */
void StartTask_SHT30(void const * argument)
{
  /* USER CODE BEGIN StartTask_SHT30 */
  /* Infinite loop */
    Task_SHT30_Process(argument);  // 调用已写好的业务层
	for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_SHT30 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartTask_EXV(void const * argument)
{
  Task_EXV_Process(argument);
  for(;;) { osDelay(1); }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

