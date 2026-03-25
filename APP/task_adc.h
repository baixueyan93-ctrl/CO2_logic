#ifndef __TASK_ADC_H
#define __TASK_ADC_H
#include <stdint.h>
extern float g_temp_10k;
extern float g_temp_50k;

// 暴露给 freertos.c 调用的主任务接口
void Task_ADC_Process(void const *argument);

#endif


