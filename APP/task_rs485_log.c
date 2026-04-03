#include "task_rs485_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp_eeprom.h"
#include "bsp_rs485.h"
#include "bsp_htc_2k.h"
#include "bsp_relay.h"
#include "sys_state.h"
#include "rtc.h"
#include "task_adc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart4;
extern osMutexId EEPROM_MutexHandle;

/* KEY 调试模式标志 */
static volatile uint8_t s_key_debug = 0;  /* 0=关, 1=开 */

// ==========================================
// ���ڽ��ջ�����
// ==========================================
uint8_t rx_byte;           // ÿ��ֻ��1���ֽ�
uint8_t rx_buffer[128];    // �����ַ���������
uint16_t rx_index = 0;     // ��ǰ�浽�˵ڼ���
volatile uint8_t rx_complete = 0; // ������ɱ�־ (1��ʾ������)

/* HAL_UART_RxCpltCallback 已统一在 bsp_inverter.c 中,
   UART4 分支处理调试串口逐字节接收 (RS485_0 口) */

// ==========================================
// ���ϼ�¼����
// ==========================================
uint8_t System_Record_Fault(uint8_t fault_code) {
    SysLog_t new_log = {0};
    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;

    // 1. ��ȡ��ǰ�� RTC ʱ���
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    new_log.Year  = sDate.Year;  new_log.Month = sDate.Month; new_log.Date  = sDate.Date;
    new_log.Hours = sTime.Hours; new_log.Minutes = sTime.Minutes; new_log.Seconds = sTime.Seconds;
    new_log.EventType = fault_code;

    // ==========================================
    // 2. ��ϵͳ�İ�ȫ�ڰ��ϳ�������������
    // ==========================================
    SysVarData_t current_sensor_data;
    SysState_GetSensor(&current_sensor_data);

    new_log.EvapTemp = current_sensor_data.VAR_EVAP_TEMP;     // 10K �����¶�
    new_log.CondTemp = current_sensor_data.VAR_EXHAUST_TEMP;  // 50K ����/�����¶� 

    // 4. ��ȫ����������־��д��
    if(osMutexWait(EEPROM_MutexHandle, 500) == osOK) {
        BSP_Log_Add(&new_log); 
        osMutexRelease(EEPROM_MutexHandle); 
        return 0; 
    }
    return 1; 
}

// ==========================================
// ����������
// ==========================================
void Task_RS485Log_Process(void const *argument) {
    
    BSP_RS485_Init();
    BSP_Log_Init(); 
    osDelay(100); 
    
    BSP_RS485_SendString("\r\n--- Simple Mode Ready! ---\r\n");
    
    // ������һ���жϽ��� (ֻ�� 1 ���ֽ�)
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);

    for(;;) {
        // ����ж�˵"��ָ������"
        if (rx_complete == 1) {
            
            // 1. ���� GET ָ��
            if(strstr((char *)rx_buffer, "GET") != NULL) {
    SysVarData_t current_data;
    SysState_GetSensor(&current_data);
    char msg[256];

    BSP_RS485_SendString("\r\n===== SENSOR DATA =====\r\n");

    /* 6路NTC温度 — 按CO2循环顺序显示 */
    sprintf(msg, "CompOut(INUI5):%.1fC  GCout(INUI6):%.1fC\r\n",
            current_data.VAR_COMP_OUT_TEMP, current_data.VAR_GC_OUT_TEMP);
    BSP_RS485_SendString(msg);
    sprintf(msg, "EvapIn(INUI0):%.1fC  EvapOut(INUI1):%.1fC  CompIn(INUI4):%.1fC\r\n",
            current_data.VAR_EVAP_IN_TEMP, current_data.VAR_EVAP_OUT_TEMP,
            current_data.VAR_COMP_IN_TEMP);
    BSP_RS485_SendString(msg);
    sprintf(msg, "Cabinet(INUI2):%.1fC\r\n",
            current_data.VAR_CABINET_TEMP);
    BSP_RS485_SendString(msg);

    /* SHT30 环境温度 */
    sprintf(msg, "Ambient(SHT30):%.1fC  Humi:%.1f%%RH\r\n",
            current_data.VAR_AMBIENT_TEMP, current_data.VAR_SHT30_HUMI);
    BSP_RS485_SendString(msg);

    /* 压力传感器 */
    sprintf(msg, "PRES  Low:%.2fbar(%.2fMPa)  High:%.2fbar(%.2fMPa)\r\n",
            current_data.VAR_SUCTION_PRES, current_data.VAR_SUCTION_PRES*0.1f,
            current_data.VAR_DISCHARGE_PRES, current_data.VAR_DISCHARGE_PRES*0.1f);
    BSP_RS485_SendString(msg);

    /* ADC原始值 (调试用) */
    float v_low  = adc_buffer[5] * 3.3f / 4095.0f;
    float v_high = adc_buffer[6] * 3.3f / 4095.0f;
    sprintf(msg, "ADC_RAW  Low:%u(%.3fV)  High:%u(%.3fV)\r\n",
            adc_buffer[5], v_low, adc_buffer[6], v_high);
    BSP_RS485_SendString(msg);

    /* CO2饱和温度 + 过热度 */
    sprintf(msg, "CO2sat Low:%.1fC  High:%.1fC  Superheat:%.1fC\r\n",
            current_data.VAR_SAT_TEMP_LOW, current_data.VAR_SAT_TEMP_HIGH,
            current_data.VAR_SUPERHEAT);
    BSP_RS485_SendString(msg);

    /* 报警标志和图标调试 */
    sprintf(msg, "ALARM: 0x%08lX  ICON0:0x%02X  ICON1:0x%02X\r\n",
            (unsigned long)g_AlarmFlags, g_IconSet.byte, g_IconSet1.byte);
    BSP_RS485_SendString(msg);

    BSP_RS485_SendString("=======================\r\n");
}
            /* KEY — 开启/关闭 按键调试模式 */
            else if(strstr((char *)rx_buffer, "KEY") != NULL) {
                s_key_debug = !s_key_debug;
                if (s_key_debug) {
                    BSP_RS485_SendString("KEY debug ON - press any key on PANEL0/PANEL1...\r\n");
                } else {
                    BSP_RS485_SendString("KEY debug OFF\r\n");
                }
}
            // 2. ���� TEST ָ��
            else if(strstr((char *)rx_buffer, "TEST") != NULL) {
                if(System_Record_Fault(0x99) == 0) BSP_RS485_SendString("Test Saved!\r\n");
            }
            // 3. ���� READ ָ��Զ�ֻ�����µ� 5 ������� 20 ����
            else if(strstr((char *)rx_buffer, "READ") != NULL) {
                BSP_RS485_SendString("\r\n--- LATEST LOG START ---\r\n");
                
                if(osMutexWait(EEPROM_MutexHandle, 1000) == osOK) {
                    SysLog_t temp_log;
                    
                    // ��ȡ�ײ�"дָ��"�ĵ�ǰλ��
                    uint8_t current_idx = BSP_Log_Get_Current_Index();
                    
                    // Ĭ����ֻ�����µļ�������ʱ�� 5 ��Ϊ��
                    uint8_t read_count = 5; 
                    
                    // �����µ�һ����ʼ����ǰ��
                    for(int i = 0; i < read_count; i++) {
                        
                        // �õ������Ļ����㷨��
                        // ΪʲôҪ�� LOG_MAX_COUNT����Ϊ��� current_idx �� 0��0-1=-1�ͳ�����
                        // �������ֵ��ȡ�࣬����������λ��ݣ�0 ����һ���� 126
                        int physical_idx = (current_idx - 1 - i + LOG_MAX_COUNT) % LOG_MAX_COUNT;
                        
                        BSP_Log_Read_By_Index(physical_idx, &temp_log); 
                        
                        // ��ѡ�Ż���������ڳ�����Χ������ 0��˵����������ȫ�µģ���ûд�룩��������
                        if (temp_log.Year == 0) continue; 
                        
                        char log_msg[128];
                        // ��ӡʱ�䡢���ϴ��롢�ڼ��� [Newest - 0], [Newest - 1]
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
            // 4. �����޸�ʱ��� SETTIME ָ��
            // Ԥ�ڸ�ʽ����: SETTIME:26-03-15,14:30:00 (��ʾ 2026��3��15�� 14ʱ30��00��)
            else if(strstr((char *)rx_buffer, "SETTIME") != NULL) {
                int year, month, date, hour, minute, second;
                
                // ʹ�� sscanf ���ַ�������ȡ���ں�ʱ��ֵ
                if (sscanf((char *)rx_buffer, "SETTIME:%d-%d-%d,%d:%d:%d", 
                           &year, &month, &date, &hour, &minute, &second) == 6) {
                    
                    RTC_TimeTypeDef sTime = {0};
                    RTC_DateTypeDef sDate = {0};
                    
                    // ����ʱ��
                    sTime.Hours = hour;
                    sTime.Minutes = minute;
                    sTime.Seconds = second;
                    sTime.TimeFormat = RTC_HOURFORMAT12_AM;
                    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
                    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
                    
                    // ��������
                    sDate.WeekDay = RTC_WEEKDAY_MONDAY; // ���ڼ��Լ��㣬Ӱ�첻���ȸ���һ
                    sDate.Month = month;
                    sDate.Date = date;
                    sDate.Year = year;
                    
                    // ��ע�⡿��STM32��Ӳ��Ҫ��������� Time������ Date
                    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK &&
                        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK) {
                        BSP_RS485_SendString("RTC Time Updated Successfully!\r\n");
                    } else {
                        BSP_RS485_SendString("ERROR: RTC Hardware Fault!\r\n");
                    }
                } else {
                    // ����û���ʽд���ˣ���ʾ��ȷ��ʽ
                    BSP_RS485_SendString("Format Error! Pls use: SETTIME:YY-MM-DD,HH:MM:SS\r\n");
                }
            }
						
            /* RELAY — 继电器控制
             *   RELAY           → 查询全部继电器状态
             *   RELAY 0 ON      → 打开蒸发风扇(K1)
             *   RELAY 3 OFF     → 关闭滑油热丝(K2)
             *   RELAY ALL OFF   → 全部关闭
             *   RELAY ALL ON    → 全部打开
             *   编号: 0=蒸发风扇 1=冷凝风扇 2=化霜热丝 3=滑油热丝 4=凝露热丝 5=照明
             */
            else if(strstr((char *)rx_buffer, "RELAY") != NULL) {
                char rmsg[128];
                char *p = strstr((char *)rx_buffer, "RELAY") + 5;

                /* 跳过空格 */
                while (*p == ' ') p++;

                if (strncmp(p, "ALL", 3) == 0) {
                    p += 3;
                    while (*p == ' ') p++;
                    if (strncmp(p, "ON", 2) == 0) {
                        for (int i = 0; i < RELAY_COUNT; i++) BSP_Relay_On((Relay_ID)i);
                        BSP_RS485_SendString("ALL relays ON\r\n");
                    } else {
                        BSP_Relay_AllOff();
                        BSP_RS485_SendString("ALL relays OFF\r\n");
                    }
                } else if (*p >= '0' && *p <= '5') {
                    int id = *p - '0';
                    p++;
                    while (*p == ' ') p++;
                    if (strncmp(p, "ON", 2) == 0) {
                        BSP_Relay_On((Relay_ID)id);
                        sprintf(rmsg, "%s(K%d) → ON\r\n", BSP_Relay_Name((Relay_ID)id), id);
                        BSP_RS485_SendString(rmsg);
                    } else if (strncmp(p, "OFF", 3) == 0) {
                        BSP_Relay_Off((Relay_ID)id);
                        sprintf(rmsg, "%s(K%d) → OFF\r\n", BSP_Relay_Name((Relay_ID)id), id);
                        BSP_RS485_SendString(rmsg);
                    } else {
                        /* 查询单个 */
                        sprintf(rmsg, "%s(K%d): %s\r\n", BSP_Relay_Name((Relay_ID)id), id,
                                BSP_Relay_GetState((Relay_ID)id) ? "ON" : "OFF");
                        BSP_RS485_SendString(rmsg);
                    }
                } else {
                    /* 无参数 — 查询全部状态 */
                    BSP_RS485_SendString("\r\n=== RELAY STATUS ===\r\n");
                    for (int i = 0; i < RELAY_COUNT; i++) {
                        sprintf(rmsg, "  [%d] %-8s : %s\r\n", i, BSP_Relay_Name((Relay_ID)i),
                                BSP_Relay_GetState((Relay_ID)i) ? "ON" : "OFF");
                        BSP_RS485_SendString(rmsg);
                    }
                    BSP_RS485_SendString("====================\r\n");
                }
            }

            // ������ϣ���ջ����������¿�ʼ����
            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));
            rx_complete = 0;
            // ==========================================
            // �������ܲ����Ĵ���Ӳ�������־λ
            // ��ֹ RS485 �շ��л�˲��ë�̵���оƬ��Ӳ����������
            // ==========================================
            __HAL_UART_CLEAR_OREFLAG(&huart4);
            __HAL_UART_CLEAR_NEFLAG(&huart4);
            __HAL_UART_CLEAR_FEFLAG(&huart4);

            huart4.ErrorCode = HAL_UART_ERROR_NONE;
            huart4.RxState = HAL_UART_STATE_READY;
            HAL_UART_Receive_IT(&huart4, &rx_byte, 1); 
        }
        
        /* KEY 调试模式: 持续扫描两个面板的按键并打印 */
        if (s_key_debug) {
            char kmsg[64];
            uint8_t k0 = HTC2K_ReadKeys();
            uint8_t k1 = HTC2K_ReadKeys1();
            if (k0 != 0x00 && k0 != 0xFF) {
                sprintf(kmsg, "[PANEL0] key=0x%02X\r\n", k0);
                BSP_RS485_SendString(kmsg);
                osDelay(200);
            }
            if (k1 != 0x00 && k1 != 0xFF) {
                sprintf(kmsg, "[PANEL1] key=0x%02X\r\n", k1);
                BSP_RS485_SendString(kmsg);
                osDelay(200);
            }
        }

        /* ============================================
         * UART 接收状态守护: 防止 Overrun/Noise/Frame 错误
         * 导致 HAL 接收中断永久停止
         * ============================================ */
        if (huart4.RxState != HAL_UART_STATE_BUSY_RX) {
            __HAL_UART_CLEAR_OREFLAG(&huart4);
            __HAL_UART_CLEAR_NEFLAG(&huart4);
            __HAL_UART_CLEAR_FEFLAG(&huart4);
            huart4.ErrorCode = HAL_UART_ERROR_NONE;
            huart4.RxState   = HAL_UART_STATE_READY;
            HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
        }

        osDelay(50);
    }
}
