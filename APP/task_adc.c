#include "task_adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "sys_state.h"

extern ADC_HandleTypeDef hadc1;

/* DMA 缓冲区, 按 Rank 顺序:
 *   [0] = INUI4  PC3  CH13  10K NTC
 *   [1] = INUI5  PC2  CH12  50K NTC
 *   [2] = INUI0  PA3  CH3   10K NTC
 *   [3] = INUI1  PA2  CH2   10K NTC
 *   [4] = INUI6  PC1  CH11  50K NTC
 */
volatile uint16_t adc_buffer[5] = {0};

float g_temp_inui4_10k = 0.0f;
float g_temp_inui5_50k = 0.0f;
float g_temp_inui0_10k = 0.0f;
float g_temp_inui1_10k = 0.0f;
float g_temp_inui6_50k = 0.0f;

/* ==========================================================
 * 10K NTC 转换函数
 * ========================================================== */
int16_t adc_to_temperature_10k(uint16_t adc_value) {
    const float V_REF = 3.3f;
    const uint16_t ADC_MAX = 4095;
    const float R_REF = 10000.0f;
    const float R_NTC_NOMINAL = 10000.0f;
    const float T_NOMINAL = 298.15f;
    const float B_VALUE = 3950.0f;

    if (adc_value > ADC_MAX) adc_value = ADC_MAX;

    float voltage = (adc_value * V_REF) / ADC_MAX;
    if (voltage < 0.001f) return 1000;

    float r_ntc = ( voltage / (V_REF - voltage)) * R_REF;
    float inv_t = (1.0f / T_NOMINAL) + (log(r_ntc / R_NTC_NOMINAL) / B_VALUE);
    float temp_celsius = (1.0f / inv_t) - 273.15f;

    int32_t temp_scaled = (int32_t)(temp_celsius * 10.0f + 0.5f);
    if (temp_scaled < -1000) return -1000;
    else if (temp_scaled > 2000) return 2000;
    else return (int16_t)temp_scaled;
}

/* ==========================================================
 * 50K NTC 转换函数
 * ========================================================== */
int16_t adc_to_temperature_50k(uint16_t adc_value) {
    const float V_REF = 3.3f;
    const uint16_t ADC_MAX = 4095;
    const float R_REF = 50000.0f;
    const float R_NTC_NOMINAL = 50000.0f;
    const float T_NOMINAL = 298.15f;
    const float B_VALUE = 3950.0f;

    if (adc_value > ADC_MAX) adc_value = ADC_MAX;

    float voltage = (adc_value * V_REF) / ADC_MAX;
    if (voltage < 0.001f) return 1000;

    float r_ntc = ( voltage / (V_REF - voltage)) * R_REF;
    float inv_t = (1.0f / T_NOMINAL) + (log(r_ntc / R_NTC_NOMINAL) / B_VALUE);
    float temp_celsius = (1.0f / inv_t) - 273.15f;

    int32_t temp_scaled = (int32_t)(temp_celsius * 10.0f + 0.5f);
    if (temp_scaled < -1000) return -1000;
    else if (temp_scaled > 2000) return 2000;
    else return (int16_t)temp_scaled;
}

/* ==========================================================
 * ADC采集线程, 持续刷新温度
 * ========================================================== */
void Task_ADC_Process(void const *argument) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 5);
    vTaskDelay(pdMS_TO_TICKS(500));

    for(;;) {
        /* 1. 转换全部5路温度 */
        int16_t t_inui4 = adc_to_temperature_10k(adc_buffer[0]);  /* INUI4 PC3 10K */
        int16_t t_inui5 = adc_to_temperature_50k(adc_buffer[1]);  /* INUI5 PC2 50K */
        int16_t t_inui0 = adc_to_temperature_10k(adc_buffer[2]);  /* INUI0 PA3 10K */
        int16_t t_inui1 = adc_to_temperature_10k(adc_buffer[3]);  /* INUI1 PA2 10K */
        int16_t t_inui6 = adc_to_temperature_50k(adc_buffer[4]);  /* INUI6 PC1 50K */

        /* 2. 更新全局温度变量 (单位 °C) */
        g_temp_inui4_10k = t_inui4 / 10.0f;
        g_temp_inui5_50k = t_inui5 / 10.0f;
        g_temp_inui0_10k = t_inui0 / 10.0f;
        g_temp_inui1_10k = t_inui1 / 10.0f;
        g_temp_inui6_50k = t_inui6 / 10.0f;

        /* 3. 写入系统状态 (互斥保护)
         *    TODO: 确认每路传感器对应的物理量后再分配
         *    目前保持原有映射: INUI4→柜温/蒸发温度, INUI5→排气温度
         */
        SysState_Lock();
        SysState_GetRawPtr()->VAR_CABINET_TEMP = g_temp_inui4_10k;
        SysState_GetRawPtr()->VAR_EVAP_TEMP    = g_temp_inui4_10k;
        SysState_GetRawPtr()->VAR_EXHAUST_TEMP = g_temp_inui5_50k;
        /* 新增3路暂不映射, 等确认用途后再分配:
         *   g_temp_inui0_10k (INUI0 PA3) → ?
         *   g_temp_inui1_10k (INUI1 PA2) → ?
         *   g_temp_inui6_50k (INUI6 PC1) → ?
         */
        SysState_Unlock();

        /* 4. 每 100ms 刷新一次 */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
