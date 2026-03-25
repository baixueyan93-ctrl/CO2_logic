#include "task_adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <math.h>     // 数学库头文件，提供 log() 函数
#include "main.h"
#include "sys_state.h"

extern ADC_HandleTypeDef hadc1; // 引用 CubeMX 生成的 ADC1 句柄

// 定义全局数组，DMA 会自动往 PC4 和 PC5 的 ADC 原始值实时搬到这里
volatile uint16_t adc_buffer[2] = {0};   
float g_temp_10k = 0.0f;
float g_temp_50k = 0.0f;
/* ==========================================================
 * 10K NTC 转换函数 (假定对应 INU4)
 * ========================================================== */
int16_t adc_to_temperature_10k(uint16_t adc_value) {
    const float V_REF = 3.3f;          
    const uint16_t ADC_MAX = 4095;     
    const float R_REF = 10000.0f;       // 电路上的参考分压电阻 (10k欧)
    const float R_NTC_NOMINAL = 10000.0f;// 10K探头在25度时的标称阻值
    const float T_NOMINAL = 298.15f;   
    const float B_VALUE = 3950.0f;     

    if (adc_value > ADC_MAX) adc_value = ADC_MAX;
    
    float voltage = (adc_value * V_REF) / ADC_MAX;
    if (voltage < 0.001f) return 1000;  // 接近0V，返回异常值
    
    float r_ntc = ( voltage / (V_REF - voltage)) * R_REF;
    float inv_t = (1.0f / T_NOMINAL) + (log(r_ntc / R_NTC_NOMINAL) / B_VALUE);
    float temp_celsius = (1.0f / inv_t) - 273.15f;

    // 放大10倍转整数，方便存储和传输
    int32_t temp_scaled = (int32_t)(temp_celsius * 10.0f + 0.5f);  
    if (temp_scaled < -1000) return -1000;
    else if (temp_scaled > 2000) return 2000;
    else return (int16_t)temp_scaled;
}

/* ==========================================================
 * 50K NTC 转换函数 (假定对应 INU5)
 * ========================================================== */
int16_t adc_to_temperature_50k(uint16_t adc_value) {
    const float V_REF = 3.3f;          
    const uint16_t ADC_MAX = 4095;     
    const float R_REF = 50000.0f;       // 电路上分压电阻固定为50k欧
    const float R_NTC_NOMINAL = 50000.0f;// 50K探头在25度时的标称阻值
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
 * ADC采集线程，不断刷新温度
 * ========================================================== */
void Task_ADC_Process(void const *argument) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
    vTaskDelay(pdMS_TO_TICKS(500)); 
    
    for(;;) {
        // 1. 计算两路温度
        int16_t t_10k_raw = adc_to_temperature_10k(adc_buffer[0]); 
        int16_t t_50k_raw = adc_to_temperature_50k(adc_buffer[1]); 
        
        // 2. 纯粹的原子锁写入 (绝对安全)
        SysState_Lock();
        float temp_10k = t_10k_raw / 10.0f;
        // 10K 探头同时给面板(柜温)和串口(蒸发温)使用
        SysState_GetRawPtr()->VAR_CABINET_TEMP = temp_10k; 
        SysState_GetRawPtr()->VAR_EVAP_TEMP    = temp_10k; 
        // 50K 探头给排气/冷凝温使用
        SysState_GetRawPtr()->VAR_EXHAUST_TEMP = t_50k_raw / 10.0f;
        SysState_Unlock();
        
        // 3. 每 100ms 刷新一次
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}



