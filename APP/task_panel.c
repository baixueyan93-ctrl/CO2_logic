#include "task_panel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "bsp_htc_2k.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * PANEL1 操作面板 — 全局状态变量
 * =========================================================================== */
float   g_set_temp  = -20.0f;  /* 用户设定温度 (初值 = SET_TEMP_TS) */
uint8_t g_panel_mode = 0;      /* 0: 正常监控, 1: 温度设置模式 */
bool    g_light_on   = false;  /* 照明灯状态 */
bool    g_system_on  = true;   /* 系统电源状态 */

/* 设置模式自动退出计时器 */
static uint32_t s_set_mode_tick = 0;
#define SET_MODE_TIMEOUT_MS  5000   /* 5秒无操作自动退出设置模式 */


/* ===========================================================================
 * PANEL0: 纯显示面板 (无按键)
 *
 * 功能:
 *   - 始终显示柜温 (VAR_CABINET_TEMP)
 *   - 图标自动反映系统状态:
 *       Def  (bit5) = 正在除霜
 *       Humi (bit6) = 满水报警
 *       Fan  (bit4) = 蒸发风机运行
 *       Ref  (bit7) = 报警指示 (有报警时闪烁)
 * =========================================================================== */
void Task_Panel0_Process(void const *argument)
{
    (void)argument;
    uint8_t blink_cnt = 0;

    HTC2K_Init();
    vTaskDelay(pdMS_TO_TICKS(200));

    for (;;) {
        /* 读取系统状态 */
        SysVarData_t sensor;
        SysState_GetSensor(&sensor);
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
        uint32_t alarms = g_AlarmFlags;

        /* --- 更新 PANEL0 图标 --- */
        g_IconSet.byte = 0;

        /* 除霜图标 */
        if (sys_bits & ST_DEFROST_ACTIVE) {
            g_IconSet.bits.Def = 1;
        }

        /* 满水图标 */
        if (sensor.VAR_LIQUID_LEVEL == 1) {
            g_IconSet.bits.Humi = 1;
        }

        /* 蒸发风机图标 */
        if (sys_bits & ST_EVAP_FAN_ON) {
            g_IconSet.bits.Fan = 1;
        }

        /* 报警指示灯 (有任何报警/警告时闪烁) */
        if (alarms & (ERR_MASK_ALL | WARN_MASK_ALL)) {
            blink_cnt++;
            if (blink_cnt & 0x04) {  /* 约200ms周期闪烁 */
                g_IconSet.bits.Ref = 1;
            }
        }

        /* 显示柜温 */
        HTC2K_ShowTemp(sensor.VAR_CABINET_TEMP);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/* ===========================================================================
 * PANEL1: 操作面板 (8按键 + 显示)
 *
 * 按键功能:
 *   1. Reset    → 复位设定温度到默认值, 清除设置模式
 *   2. Set      → 进入/退出温度设置模式
 *   3. Up       → 设置模式: 温度+0.5℃
 *   4. Down     → 设置模式: 温度-0.5℃
 *   5. Defrost  → 手动触发除霜
 *   6. Light    → 切换照明灯开关
 *   7. Inspect  → 保留 (预留功能)
 *   8. Power    → 系统电源开关
 *
 * 显示:
 *   正常模式 → 显示柜温
 *   设置模式 → 显示设定温度 (Set图标亮)
 * =========================================================================== */
void Task_Panel1_Process(void const *argument)
{
    (void)argument;

    HTC2K_Init1();
    vTaskDelay(pdMS_TO_TICKS(200));

    g_set_temp = SET_TEMP_TS;  /* 初始化为配置文件默认值 */

    for (;;) {
        /* 读取系统状态 */
        SysVarData_t sensor;
        SysState_GetSensor(&sensor);
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

        /* ============================================
         * 按键扫描
         * ============================================ */
        uint8_t key = HTC2K_ReadKeys1();

        if (key != 0x00 && key != 0xFF) {

            /* --- 1. Reset键: 一键复位 --- */
            if (key == KEY_CODE_RST) {
                g_panel_mode = 0;
                g_set_temp = SET_TEMP_TS;
                g_IconSet1.bits.Set = 0;
            }

            /* --- 2. Set键: 进入/退出设置模式 --- */
            if (key == KEY_CODE_SET) {
                if (g_panel_mode == 0) {
                    g_panel_mode = 1;
                    g_IconSet1.bits.Set = 1;
                    s_set_mode_tick = xTaskGetTickCount();
                } else {
                    g_panel_mode = 0;
                    g_IconSet1.bits.Set = 0;
                }
            }

            /* --- 3/4. 上下键: 调温 (仅设置模式) --- */
            if (g_panel_mode == 1) {
                if (key == KEY_CODE_UP) {
                    g_set_temp += 0.5f;
                    s_set_mode_tick = xTaskGetTickCount();
                }
                if (key == KEY_CODE_DOWN) {
                    g_set_temp -= 0.5f;
                    s_set_mode_tick = xTaskGetTickCount();
                }
                /* 温度范围限制 */
                if (g_set_temp > 20.0f)  g_set_temp = 20.0f;
                if (g_set_temp < -30.0f) g_set_temp = -30.0f;
            }

            /* --- 5. 一键除霜键 --- */
            if (key == KEY_CODE_DEFROST) {
                /* 手动触发除霜: 置位除霜标志, 由除霜任务响应 */
                if (sys_bits & ST_COMP_RUNNING) {
                    xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE);
                }
            }

            /* --- 6. 照明键: 切换照明灯 --- */
            if (key == KEY_CODE_LIGHT) {
                g_light_on = !g_light_on;
                /* TODO: 调用照明灯 BSP 接口 (如 BSP_Light_Set(g_light_on)) */
            }

            /* --- 7. 点检键: 保留 --- */
            if (key == KEY_CODE_INSPECT) {
                /* 保留, 后续扩展功能 */
            }

            /* --- 8. 电源开关键 --- */
            if (key == KEY_CODE_POWER) {
                g_system_on = !g_system_on;
                if (g_system_on) {
                    xEventGroupSetBits(SysEventGroup, ST_SYSTEM_ON);
                } else {
                    xEventGroupClearBits(SysEventGroup, ST_SYSTEM_ON);
                    /* 关机: 清除压缩机运行状态 */
                    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
                }
            }

            /* 按键消抖延时 */
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        /* ============================================
         * 设置模式超时自动退出 (5秒无操作)
         * ============================================ */
        if (g_panel_mode == 1) {
            uint32_t elapsed = xTaskGetTickCount() - s_set_mode_tick;
            if (elapsed > pdMS_TO_TICKS(SET_MODE_TIMEOUT_MS)) {
                g_panel_mode = 0;
                g_IconSet1.bits.Set = 0;
            }
        }

        /* ============================================
         * 更新 PANEL1 图标
         * ============================================ */
        /* 保留 Set 位的当前状态, 更新其他图标 */
        uint8_t set_bit = g_IconSet1.bits.Set;
        g_IconSet1.byte = 0;
        g_IconSet1.bits.Set = set_bit;

        /* 制冷图标 */
        if (sys_bits & ST_COMP_RUNNING) {
            g_IconSet1.bits.Ref = 1;
        }

        /* 风扇图标 */
        if (sys_bits & ST_EVAP_FAN_ON) {
            g_IconSet1.bits.Fan = 1;
        }

        /* 除霜图标 */
        if (sys_bits & ST_DEFROST_ACTIVE) {
            g_IconSet1.bits.Def = 1;
        }

        /* 照明图标 */
        if (g_light_on) {
            g_IconSet1.bits.Light = 1;
        }

        /* 加热图标 (除霜加热阶段) */
        if (sys_bits & ST_DEF_HEATING) {
            g_IconSet1.bits.Heat = 1;
        }

        /* ============================================
         * 显示
         * ============================================ */
        /* 正常模式和设置模式都显示目标温度 (PANEL1不显示柜温) */
        HTC2K_ShowTemp1(g_set_temp);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/* ===========================================================================
 * 兼容旧接口: Task_Panel_Process
 *
 * 原本只有一个面板任务, 现在拆分为 PANEL0 + PANEL1.
 * 保留此函数供 freertos.c 中原有的 StartTask03() 调用,
 * 实际执行 PANEL0 (纯显示) 逻辑.
 * PANEL1 需要在 freertos.c 中新建独立任务.
 * =========================================================================== */
void Task_Panel_Process(void const *argument)
{
    Task_Panel0_Process(argument);
}
