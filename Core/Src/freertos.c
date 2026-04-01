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
#include "task_led.h"        /* LED 指示灯任务 */
//#include "task_buzzer.h"   /* 蜂鸣器任务 (暂未启用) */
#include "task_panel.h"      /* 面板显示+按键任务 */
#include "task_rs485_log.h"  /* RS485 通信/日志任务 */
//#include "task_XKC_Y20_V.h" /* XKC_Y20_V 液位计任务 (暂未启用) */
#include "task_adc.h"        /* ADC 采集任务 (NTC温度+压力) */
#include "task_sht30.h"      /* SHT30 温湿度采集任务 */
#include "task_exv.h"        /* 电子膨胀阀测试任务 (已由FreqExv替代) */
#include "task_temp_ctrl.h"  /* 温度控制流程 (逻辑图1) */
#include "task_defrost.h"    /* 除霜流程 (逻辑图2) */
#include "task_evap_fan.h"   /* 蒸发风机(1控2)流程 (逻辑图3) */
#include "task_freq_exv.h"   /* 变频控制(PID)+膨胀阀流程 (逻辑图4) */
#include "task_timer_svc.h"  /* 定时中断服务 (逻辑图5) */
#include "task_cond_fan.h"   /* 冷凝风机(1控3)流程 (逻辑图6) */
#include "bsp_i2c_mutex.h"   /* I2C1 总线互斥锁 */
#include "bsp_relay.h"       /* 6路继电器驱动 */
#include "sys_state.h"       /* 系统状态全局变量 */
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
osThreadId Task_TempCtrlHandle;
osThreadId Task_DefrostHandle;
osThreadId Task_EvapFanHandle;
osThreadId Task_FreqExvHandle;
osThreadId Task_TimerSvcHandle;
osThreadId Task_CondFanHandle;
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
void StartTask_TempCtrl(void const * argument);
void StartTask_Defrost(void const * argument);
void StartTask_EvapFan(void const * argument);
void StartTask_FreqExv(void const * argument);
void StartTask_TimerSvc(void const * argument);
void StartTask_CondFan(void const * argument);

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
  BSP_I2C1_MutexInit();  /* 优先初始化硬件互斥锁 */
  BSP_Relay_Init();      /* 继电器GPIO初始化, 上电默认全部OFF */
  SysState_Init();       /* 在任务启动前初始化全局变量和系统锁 */
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

  /* 定时中断服务任务 (逻辑图5, 高于普通优先级, 必须先启动, 其他任务依赖定时器) */
  osThreadDef(Task_TimerSvc, StartTask_TimerSvc, osPriorityAboveNormal, 0, 256);
  Task_TimerSvcHandle = osThreadCreate(osThread(Task_TimerSvc), NULL);

  /* 温度控制主任务 (逻辑图1, 普通优先级, 512栈) */
  osThreadDef(Task_TempCtrl, StartTask_TempCtrl, osPriorityNormal, 0, 512);
  Task_TempCtrlHandle = osThreadCreate(osThread(Task_TempCtrl), NULL);

  /* 除霜控制任务 (逻辑图2, 普通优先级, 512栈) */
  osThreadDef(Task_Defrost, StartTask_Defrost, osPriorityNormal, 0, 512);
  Task_DefrostHandle = osThreadCreate(osThread(Task_Defrost), NULL);

  /* 蒸发风机(1控2)任务 (逻辑图3, 普通优先级, 256栈) */
  osThreadDef(Task_EvapFan, StartTask_EvapFan, osPriorityNormal, 0, 256);
  Task_EvapFanHandle = osThreadCreate(osThread(Task_EvapFan), NULL);

  /* 变频控制(PID)+电子膨胀阀任务 (逻辑图4, 普通优先级, 512栈) */
  osThreadDef(Task_FreqExv, StartTask_FreqExv, osPriorityNormal, 0, 512);
  Task_FreqExvHandle = osThreadCreate(osThread(Task_FreqExv), NULL);

  /* 冷凝风机(1控3)任务 (逻辑图6, 普通优先级, 256栈) */
  osThreadDef(Task_CondFan, StartTask_CondFan, osPriorityNormal, 0, 256);
  Task_CondFanHandle = osThreadCreate(osThread(Task_CondFan), NULL);

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
  Task_LED_Process(argument);  /* 调用外部LED业务函数 */
  for(;;) { osDelay(1); }     /* 安全兜底 */
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
  Task_RS485Log_Process(argument);  /* 调用RS485通信业务 */
  for(;;) { osDelay(1); }          /* 安全兜底 */
  /* USER CODE END StartTask_RS485 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the TaskPanel thread.
* @param argument: Not used
* @retval None
*/
extern void Task_Panel_Process(void const *argument);
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  Task_Panel_Process(argument);  /* 调用面板显示+按键业务 */
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
  Task_ADC_Process(argument);  /* ADC采集: 5路NTC + 2路压力 */
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
  Task_SHT30_Process(argument);  /* SHT30温湿度采集, 写入柜温 */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_SHT30 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ===== 6大逻辑任务入口 ===== */

void StartTask_EXV(void const * argument)
{
  Task_EXV_Process(argument);    /* 膨胀阀测试任务 (已由FreqExv替代) */
  for(;;) { osDelay(1); }
}

void StartTask_TempCtrl(void const * argument)
{
  Task_TempCtrl_Process(argument);  /* 逻辑图1: 温度控制 */
  for(;;) { osDelay(1); }
}

void StartTask_Defrost(void const * argument)
{
  Task_Defrost_Process(argument);   /* 逻辑图2: 除霜控制 */
  for(;;) { osDelay(1); }
}

void StartTask_EvapFan(void const * argument)
{
  Task_EvapFan_Process(argument);   /* 逻辑图3: 蒸发风机(1控2) */
  for(;;) { osDelay(1); }
}

void StartTask_FreqExv(void const * argument)
{
  Task_FreqExv_Process(argument);   /* 逻辑图4: 变频PID+膨胀阀 */
  for(;;) { osDelay(1); }
}

void StartTask_TimerSvc(void const * argument)
{
  Task_TimerSvc_Process(argument);  /* 逻辑图5: 定时中断服务 */
  for(;;) { osDelay(1); }
}

void StartTask_CondFan(void const * argument)
{
  Task_CondFan_Process(argument);   /* 逻辑图6: 冷凝风机(1控3) */
  for(;;) { osDelay(1); }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
