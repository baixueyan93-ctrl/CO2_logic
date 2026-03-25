#include "task_rs485_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp_eeprom.h"
#include "bsp_rs485.h"
#include "sys_state.h"
#include "rtc.h"
#include "task_adc.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart4;
extern osMutexId EEPROM_MutexHandle;

// ==========================================
// 串口接收缓冲区
// ==========================================
uint8_t rx_byte;           // 每次只收1个字节
uint8_t rx_buffer[128];    // 完整字符串缓冲区
uint16_t rx_index = 0;     // 当前存到了第几个
volatile uint8_t rx_complete = 0; // 接收完成标志 (1表示收完了)

// HAL 串口的接收中断回调 (每收到1个字节，自动调用这个函数)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == UART4) {
        rx_buffer[rx_index++] = rx_byte;
        
        // 如果收到回车换行，或者缓冲区快装满了，认为收到了一条完整指令
        if(rx_byte == '\n' || rx_byte == '\r' || rx_index >= 127) {
            rx_buffer[rx_index] = '\0'; // 加上字符串结尾
            rx_complete = 1;            // 通知任务去处理
        } else {
            // 还没收完，就继续监听下一个字节
            HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
        }
    }
}

// ==========================================
// 故障记录函数
// ==========================================
uint8_t System_Record_Fault(uint8_t fault_code) {
    SysLog_t new_log = {0};
    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;

    // 1. 获取当前的 RTC 时间戳
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    new_log.Year  = sDate.Year;  new_log.Month = sDate.Month; new_log.Date  = sDate.Date;
    new_log.Hours = sTime.Hours; new_log.Minutes = sTime.Minutes; new_log.Seconds = sTime.Seconds;
    new_log.EventType = fault_code;

    // ==========================================
    // 2. 从系统的安全黑板上抄出传感器数据
    // ==========================================
    SysVarData_t current_sensor_data;
    SysState_GetSensor(&current_sensor_data);

    new_log.EvapTemp = current_sensor_data.VAR_EVAP_TEMP;     // 10K 蒸发温度
    new_log.CondTemp = current_sensor_data.VAR_EXHAUST_TEMP;  // 50K 排气/冷凝温度 

    // 4. 安全地往故障日志环写入
    if(osMutexWait(EEPROM_MutexHandle, 500) == osOK) {
        BSP_Log_Add(&new_log); 
        osMutexRelease(EEPROM_MutexHandle); 
        return 0; 
    }
    return 1; 
}

// ==========================================
// 任务主函数
// ==========================================
void Task_RS485Log_Process(void const *argument) {
    
    BSP_RS485_Init();
    BSP_Log_Init(); 
    osDelay(100); 
    
    BSP_RS485_SendString("\r\n--- Simple Mode Ready! ---\r\n");
    
    // 启动第一次中断接收 (只收 1 个字节)
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
    
    for(;;) {
        // 如果中断说"有指令来了"
        if (rx_complete == 1) {
            
            // 1. 处理 GET 指令
            if(strstr((char *)rx_buffer, "GET") != NULL) {
    SysVarData_t current_data;
    
    // 从系统的安全黑板上读取传感器数据
    SysState_GetSensor(&current_data); 
    
    char reply_msg[128];
    // 打印从结构体中取出的两路温度(10K)和排气温度(50K)
    sprintf(reply_msg, "T_Evap(10K):%.1f | T_Exh(50K):%.1f | SHT30: %.1fC %.1f%%RH\r\n", 
            current_data.VAR_EVAP_TEMP, current_data.VAR_EXHAUST_TEMP,
            current_data.VAR_SHT30_TEMP, current_data.VAR_SHT30_HUMI);
            
    BSP_RS485_SendString(reply_msg);
}
            // 2. 处理 TEST 指令
            else if(strstr((char *)rx_buffer, "TEST") != NULL) {
                if(System_Record_Fault(0x99) == 0) BSP_RS485_SendString("Test Saved!\r\n");
            }
            // 3. 处理 READ 指令（自动只读最新的 5 条，最多 20 条）
            else if(strstr((char *)rx_buffer, "READ") != NULL) {
                BSP_RS485_SendString("\r\n--- LATEST LOG START ---\r\n");
                
                if(osMutexWait(EEPROM_MutexHandle, 1000) == osOK) {
                    SysLog_t temp_log;
                    
                    // 获取底层"写指针"的当前位置
                    uint8_t current_idx = BSP_Log_Get_Current_Index();
                    
                    // 默认先只看最新的几条，暂时以 5 条为例
                    uint8_t read_count = 5; 
                    
                    // 从最新的一条开始，往前翻
                    for(int i = 0; i < read_count; i++) {
                        
                        // 用倒着数的环形算法找
                        // 为什么要加 LOG_MAX_COUNT：因为如果 current_idx 是 0，0-1=-1就出界了
                        // 加上最大值再取余，完美解决环形回溯：0 的上一条是 126
                        int physical_idx = (current_idx - 1 - i + LOG_MAX_COUNT) % LOG_MAX_COUNT;
                        
                        BSP_Log_Read_By_Index(physical_idx, &temp_log); 
                        
                        // 可选优化：如果日期超出范围或者是 0，说明这条还是全新的（还没写入），就跳过
                        if (temp_log.Year == 0) continue; 
                        
                        char log_msg[128];
                        // 打印时间、故障代码、第几条 [Newest - 0], [Newest - 1]
                        sprintf(log_msg, "[Newest-%d] (Idx:%d) 20%02d-%02d-%02d %02d:%02d:%02d | Evt:0x%02X | Temp:%.1f\r\n", 
                                i, physical_idx, temp_log.Year, temp_log.Month, temp_log.Date,
                                temp_log.Hours, temp_log.Minutes, temp_log.Seconds,
                                temp_log.EventType, temp_log.EvapTemp);
                                
                        BSP_RS485_SendString(log_msg);
                        osDelay(10); 
                    }
                    osMutexRelease(EEPROM_MutexHandle); 
                }
                BSP_RS485_SendString("--- LATEST LOG END ---\r\n");
            }
            // 4. 处理修改时间的 SETTIME 指令
            // 预期格式例子: SETTIME:26-03-15,14:30:00 (表示 2026年3月15日 14时30分00秒)
            else if(strstr((char *)rx_buffer, "SETTIME") != NULL) {
                int year, month, date, hour, minute, second;
                
                // 使用 sscanf 从字符串中提取日期和时间值
                if (sscanf((char *)rx_buffer, "SETTIME:%d-%d-%d,%d:%d:%d", 
                           &year, &month, &date, &hour, &minute, &second) == 6) {
                    
                    RTC_TimeTypeDef sTime = {0};
                    RTC_DateTypeDef sDate = {0};
                    
                    // 设置时间
                    sTime.Hours = hour;
                    sTime.Minutes = minute;
                    sTime.Seconds = second;
                    sTime.TimeFormat = RTC_HOURFORMAT12_AM;
                    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
                    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
                    
                    // 设置日期
                    sDate.WeekDay = RTC_WEEKDAY_MONDAY; // 星期几自己算，影响不大，先给周一
                    sDate.Month = month;
                    sDate.Date = date;
                    sDate.Year = year;
                    
                    // 【注意】在STM32上硬件要求必须先设 Time，再设 Date
                    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK &&
                        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK) {
                        BSP_RS485_SendString("RTC Time Updated Successfully!\r\n");
                    } else {
                        BSP_RS485_SendString("ERROR: RTC Hardware Fault!\r\n");
                    }
                } else {
                    // 如果用户格式写错了，提示正确格式
                    BSP_RS485_SendString("Format Error! Pls use: SETTIME:YY-MM-DD,HH:MM:SS\r\n");
                }
            }
						
            // 处理完毕，清空缓冲区，重新开始接收
            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_complete = 0;
            // ==========================================
            // 清理可能残留的串口硬件错误标志位
            // 防止 RS485 收发切换瞬间毛刺导致芯片的硬件保护挂起
            // ==========================================
            __HAL_UART_CLEAR_OREFLAG(&huart4); // 清除溢出错误 (Overrun)
            __HAL_UART_CLEAR_NEFLAG(&huart4);  // 清除噪声错误 (Noise)
            __HAL_UART_CLEAR_FEFLAG(&huart4);  // 清除帧错误 (Framing)
            
            huart4.ErrorCode = HAL_UART_ERROR_NONE; // 强制骗过 HAL 库，说没有错误
            huart4.RxState = HAL_UART_STATE_READY;  // 强行把状态恢复到"准备接收"状态
            HAL_UART_Receive_IT(&huart4, &rx_byte, 1); 
        }
        
        // 没事干就睡 50ms，不占 CPU
        osDelay(50);
    }
}



