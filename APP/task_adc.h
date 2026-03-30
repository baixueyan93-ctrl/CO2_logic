#ifndef __TASK_ADC_H
#define __TASK_ADC_H
#include <stdint.h>

/* ADC 通道温度值 (°C, 已除以10) */
extern float g_temp_inui4_10k;   /* INUI4  PC3  10K NTC (原 g_temp_10k) */
extern float g_temp_inui5_50k;   /* INUI5  PC2  50K NTC (原 g_temp_50k) */
extern float g_temp_inui0_10k;   /* INUI0  PA3  10K NTC (新增) */
extern float g_temp_inui1_10k;   /* INUI1  PA2  10K NTC (新增) */
extern float g_temp_inui6_50k;   /* INUI6  PC1  50K NTC (新增) */

/* 兼容旧变量名 */
#define g_temp_10k  g_temp_inui4_10k
#define g_temp_50k  g_temp_inui5_50k

void Task_ADC_Process(void const *argument);

#endif


