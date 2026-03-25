#include "task_panel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_htc_2k.h"
#include "sys_state.h"
#include "sys_config.h"

float g_env_temp  = -5.0f;
float g_set_limit = -5.0f;
uint8_t g_mode    = 0;       // 0: Monitor, 1: Setting

void Task_Panel_Process(void const *argument) {
    uint8_t key_val = 0;
    
    HTC2K_Init();
    
    for(;;) {
        /* Read sensor */
        SysVarData_t sensor_data;
        SysState_GetSensor(&sensor_data);
        g_env_temp = sensor_data.VAR_CABINET_TEMP;
        
        /* Key scan */
        key_val = HTC2K_ReadKeys();
        
        if (key_val != 0x00 && key_val != 0xFF) {
            
            if (key_val == KEY_CODE_SET) {
                if (g_mode == 0) {
                    g_mode = 1;
                    g_IconSet.bits.Set = 1;
                } else {
                    g_mode = 0;
                    g_IconSet.bits.Set = 0;
                }
            }
            
            if (g_mode == 1) {
                if (key_val == KEY_CODE_UP)   g_set_limit += 0.5f; 
                if (key_val == KEY_CODE_DOWN) g_set_limit -= 0.5f; 
                if(g_set_limit > 30.0f) g_set_limit = 30.0f;
                if(g_set_limit < -30.0f) g_set_limit = -30.0f;
            }
            
            if (key_val == KEY_CODE_RST) {
                g_mode = 0;
                g_IconSet.bits.Set = 0;
                g_set_limit = -5.0f;
            }
            
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }

        /* Display */
        if (g_mode == 0) {
            if (g_env_temp > (g_set_limit + SET_TEMP_HYST_C1)) {
                g_IconSet.bits.Ref = 1;  
                g_IconSet.bits.Fan = 1;  
                g_IconSet.bits.Heat = 0; 
            } else if (g_env_temp < (g_set_limit - SET_TEMP_HYST_C1)) {
                g_IconSet.bits.Ref = 0;
                g_IconSet.bits.Fan = 0;
                g_IconSet.bits.Heat = 1; 
            }
            HTC2K_ShowTemp(g_env_temp);
        } else {
            HTC2K_ShowTemp(g_set_limit);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}




