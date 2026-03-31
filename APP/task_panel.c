#include "task_panel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "bsp_htc_2k.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 面板操作全局变量
 * =========================================================================== */
float   g_set_temp  = -20.0f;  /* 用户设定温度 */
uint8_t g_panel_mode = 0;      /* 0: 正常监控, 1: 温度设置模式 */
bool    g_light_on   = false;  /* 照明灯状态 */
bool    g_system_on  = true;   /* 系统电源状态 */

static uint32_t s_set_mode_tick = 0;
#define SET_MODE_TIMEOUT_MS  5000   /* 5秒无操作自动退出设置模式 */

/* ===========================================================================
 * 面板总任务：一个任务同时驱动 PANEL0(纯显示) 和 PANEL1(带按键)
 * =========================================================================== */
void Task_Panel_Process(void const *argument)
{
    (void)argument;
    uint8_t blink_cnt = 0;

    /* 1. 同时初始化两个面板的底层引脚 */
    HTC2K_Init();   // 初始化 PANEL0 (PB6/PB7)
    HTC2K_Init1();  // 初始化 PANEL1 (PB4/PB5)
    vTaskDelay(pdMS_TO_TICKS(200));

    g_set_temp = SET_TEMP_TS;  /* 初始化为配置文件默认值 */

    for (;;) {
        /* 读取系统状态数据 */
        SysVarData_t sensor;
        SysState_GetSensor(&sensor);
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
        uint32_t alarms = g_AlarmFlags;

        /* ============================================
         * 模块一：处理 PANEL0 (纯显示屏，无按键)
         * ============================================ */
        g_IconSet.byte = 0;
        if (sys_bits & ST_DEFROST_ACTIVE) g_IconSet.bits.Def = 1;
        if (sensor.VAR_LIQUID_LEVEL == 1) g_IconSet.bits.Humi = 1;
        if (sys_bits & ST_EVAP_FAN_ON)    g_IconSet.bits.Fan = 1;
        if (alarms & (ERR_MASK_ALL | WARN_MASK_ALL)) {
            blink_cnt++;
            if (blink_cnt & 0x04) g_IconSet.bits.Ref = 1; // 报警时图标闪烁
        }

        /* 【修改点】PANEL0 永远只专心显示 SHT-30 的温度 */
        HTC2K_ShowTemp(sensor.VAR_SHT30_TEMP);


        /* ============================================
         * 模块二：处理 PANEL1 (操作屏，带按键)
         * ============================================ */
        uint8_t key = HTC2K_ReadKeys1(); // 只扫描 PANEL1 的按键

        if (key != 0x00 && key != 0xFF) {
            if (key == KEY_CODE_RST) {
                g_panel_mode = 0;
                g_set_temp = SET_TEMP_TS;
            }
            else if (key == KEY_CODE_SET) {
                g_panel_mode = !g_panel_mode; // 切换设置模式
                s_set_mode_tick = xTaskGetTickCount();
            }
            else if (g_panel_mode == 1) {
                if (key == KEY_CODE_UP) { g_set_temp += 0.5f; s_set_mode_tick = xTaskGetTickCount(); }
                if (key == KEY_CODE_DOWN){ g_set_temp -= 0.5f; s_set_mode_tick = xTaskGetTickCount(); }
                if (g_set_temp > 20.0f)  g_set_temp = 20.0f;
                if (g_set_temp < -30.0f) g_set_temp = -30.0f;
            }
            else if (key == KEY_CODE_DEFROST) {
                if (sys_bits & ST_COMP_RUNNING) xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE);
            }
            else if (key == KEY_CODE_LIGHT) {
                g_light_on = !g_light_on;
            }
            else if (key == KEY_CODE_POWER) {
                g_system_on = !g_system_on;
                if (g_system_on) xEventGroupSetBits(SysEventGroup, ST_SYSTEM_ON);
                else {
                    xEventGroupClearBits(SysEventGroup, ST_SYSTEM_ON);
                    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(150)); // 按键消抖
        }

        /* 5秒无操作自动退出设置模式 */
        if (g_panel_mode == 1) {
            if ((xTaskGetTickCount() - s_set_mode_tick) > pdMS_TO_TICKS(SET_MODE_TIMEOUT_MS)) {
                g_panel_mode = 0;
            }
        }

        /* 更新 PANEL1 的指示灯图标 */
        g_IconSet1.byte = 0;
        if (g_panel_mode == 1)            g_IconSet1.bits.Set = 1;
        if (sys_bits & ST_COMP_RUNNING)   g_IconSet1.bits.Ref = 1;
        if (sys_bits & ST_EVAP_FAN_ON)    g_IconSet1.bits.Fan = 1;
        if (sys_bits & ST_DEFROST_ACTIVE) g_IconSet1.bits.Def = 1;
        if (g_light_on)                   g_IconSet1.bits.Light = 1;
        if (sys_bits & ST_DEF_HEATING)    g_IconSet1.bits.Heat = 1;

        /* PANEL1 显示内容判断 */
        if (g_panel_mode == 0) {
            /* 【确认点】正常模式下，PANEL1 也显示 SHT-30 环境温度 */
            HTC2K_ShowTemp1(sensor.VAR_SHT30_TEMP);
        } else {
            /* 设置模式显示目标设定温度 */
            HTC2K_ShowTemp1(g_set_temp);
        }

        /* 任务休眠 50ms，释放 CPU 给其他任务 */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
