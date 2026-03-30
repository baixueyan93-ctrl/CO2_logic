#ifndef __TASK_ADC_H
#define __TASK_ADC_H
#include <stdint.h>

/* ADC 通道温度值 (单位 °C) */
extern float g_temp_inui4_10k;   /* INUI4  PC3  10K NTC (原 g_temp_10k) */
extern float g_temp_inui5_50k;   /* INUI5  PC2  50K NTC (原 g_temp_50k) */
extern float g_temp_inui0_10k;   /* INUI0  PA3  10K NTC */
extern float g_temp_inui1_10k;   /* INUI1  PA2  10K NTC */
extern float g_temp_inui6_50k;   /* INUI6  PC1  50K NTC */

/* 压力传感器 (单位 bar) */
extern float g_pres_low;         /* AN5VIN0 PA7  低压 SANHUA YCQB09L02 0~9MPa  */
extern float g_pres_high;        /* AN5VIN1 PC4  高压 SANHUA YCQB15L01 0~15MPa */

/* 兼容旧变量名 */
#define g_temp_10k  g_temp_inui4_10k
#define g_temp_50k  g_temp_inui5_50k

/* CO2 饱和压力→饱和温度 转换 (供外部模块调用) */
float CO2_PressureToSatTemp(float pressure_bar);

void Task_ADC_Process(void const *argument);

#endif
