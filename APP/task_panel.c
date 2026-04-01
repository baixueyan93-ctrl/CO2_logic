#include "task_panel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "bsp_htc_2k.h"
#include "bsp_relay.h"
#include "bsp_inverter.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 面板操作全局变量
 * =========================================================================== */
float   g_set_temp   = SET_TEMP_TS;  /* 用户设定温度, 默认跟随Ts */
uint8_t g_panel_mode = 0;       /* 0: 正常监控, 1: 温度设置模式 */
bool    g_light_on   = false;   /* 照明灯状态 */
bool    g_system_on  = true;    /* 系统电源状态 */

static uint32_t s_set_mode_tick = 0;
#define SET_MODE_TIMEOUT_MS  5000   /* 5秒无操作自动退出设置模式 */

/* ===========================================================================
 * 双面板任务
 *
 * PANEL0 (PB6/PB7): 显示温度+图标 + 4按键 (Reset/Set/上/下)
 * PANEL1 (PB4/PB5): 显示温度+图标 + 4按键 (除霜/照明/点检/电源)
 *
 * 两块屏显示内容相同: 正常模式显示SHT30温度, 设置模式显示设定温度
 * 图标也相同: 除霜/风扇/制冷/加热/灯光等
 * =========================================================================== */
void Task_Panel_Process(void const *argument)
{
    (void)argument;
    uint8_t blink_cnt = 0;

    /* 初始化两个面板 */
    HTC2K_Init();   /* PANEL0 (PB6/PB7) */
    HTC2K_Init1();  /* PANEL1 (PB4/PB5) */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* g_set_temp 已在全局初始化为 SET_TEMP_TS (-10℃) */

    for (;;) {
        /* 读取系统状态 */
        SysVarData_t sensor;
        SysState_GetSensor(&sensor);
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
        uint32_t alarms = g_AlarmFlags;

        /* ============================================
         * 构建统一图标 (两块屏共用)
         * ============================================ */
        uint8_t icons = 0;
        icon_type_t icon_common;
        icon_common.byte = 0;

        if (sys_bits & ST_DEFROST_ACTIVE) icon_common.bits.Def   = 1;
        if (sys_bits & ST_DEF_HEATING)    icon_common.bits.Heat  = 1;
        if (sys_bits & ST_EVAP_FAN_ON)    icon_common.bits.Fan   = 1;
        if (sys_bits & ST_COMP_RUNNING)   icon_common.bits.Ref   = 1;
        if (g_light_on)                   icon_common.bits.Light = 1;
        if (g_panel_mode == 1)            icon_common.bits.Set   = 1;
        if (sensor.VAR_LIQUID_LEVEL == 1) icon_common.bits.Humi  = 1;

        /* 报警时制冷图标闪烁 */
        if (alarms & (ERR_MASK_ALL | WARN_MASK_ALL)) {
            blink_cnt++;
            if (blink_cnt & 0x04) icon_common.bits.Ref = 1;
            else                  icon_common.bits.Ref = 0;
        }

        /* 两块屏图标相同 */
        g_IconSet.byte  = icon_common.byte;
        g_IconSet1.byte = icon_common.byte;

        /* ============================================
         * 显示温度 (两块屏内容相同)
         * ============================================ */
        if (g_panel_mode == 0) {
            /* 正常模式: 显示 SHT-30 环境温度 */
            HTC2K_ShowTemp(sensor.VAR_SHT30_TEMP);
            HTC2K_ShowTemp1(sensor.VAR_SHT30_TEMP);
        } else {
            /* 设置模式: 显示设定温度 */
            HTC2K_ShowTemp(g_set_temp);
            HTC2K_ShowTemp1(g_set_temp);
        }

        /* ============================================
         * PANEL0 按键: Reset / Set / 上 / 下
         * ============================================ */
        uint8_t key0 = HTC2K_ReadKeys();

        if (key0 != 0x00 && key0 != 0xFF) {
            if (key0 == KEY_CODE_RST) {
                /* 复位: 退出设置模式, 恢复默认温度 */
                g_panel_mode = 0;
                g_set_temp = SET_TEMP_TS;  /* 恢复默认温度 */
            }
            else if (key0 == KEY_CODE_SET) {
                /* 切换设置模式 */
                g_panel_mode = !g_panel_mode;
                s_set_mode_tick = xTaskGetTickCount();
            }
            else if (g_panel_mode == 1) {
                /* 设置模式下: 上/下调温 */
                if (key0 == KEY_CODE_UP)   { g_set_temp += 0.5f; s_set_mode_tick = xTaskGetTickCount(); }
                if (key0 == KEY_CODE_DOWN) { g_set_temp -= 0.5f; s_set_mode_tick = xTaskGetTickCount(); }
                if (g_set_temp > 20.0f)  g_set_temp = 20.0f;
                if (g_set_temp < -30.0f) g_set_temp = -30.0f;
            }
            vTaskDelay(pdMS_TO_TICKS(150));  /* 消抖 */
        }

        /* ============================================
         * PANEL1 按键: 除霜 / 照明 / 点检 / 电源
         * ============================================ */
        uint8_t key1 = HTC2K_ReadKeys1();

        if (key1 != 0x00 && key1 != 0xFF) {
            if (key1 == KEY_CODE_DEFROST) {
                /* 一键除霜: 再按取消 */
                if (sys_bits & ST_DEFROST_ACTIVE) {
                    xEventGroupClearBits(SysEventGroup,
                        ST_DEFROST_ACTIVE | ST_DEF_HEATING | ST_DEF_DRIPPING);
                } else {
                    xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE);
                }
            }
            else if (key1 == KEY_CODE_LIGHT) {
                /* 照明开关 */
                g_light_on = !g_light_on;
                BSP_Relay_Set(RELAY_LIGHT, g_light_on ? 1 : 0);
            }
            else if (key1 == KEY_CODE_INSPECT) {
                /* 点检键: 预留, 后续进入日志模式 */
            }
            else if (key1 == KEY_CODE_POWER) {
                /* 电源开关 */
                g_system_on = !g_system_on;
                if (g_system_on) {
                    xEventGroupSetBits(SysEventGroup, ST_SYSTEM_ON);
                    BSP_Inverter_Send(0x01, (uint16_t)SET_FREQ_INIT);  /* 开机, 初始频率80Hz */
                } else {
                    BSP_Inverter_Send(0x00, 0);                         /* 关机 */
                    xEventGroupClearBits(SysEventGroup, ST_SYSTEM_ON);
                    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(150));  /* 消抖 */
        }

        /* 5秒无操作自动退出设置模式 */
        if (g_panel_mode == 1) {
            if ((xTaskGetTickCount() - s_set_mode_tick) > pdMS_TO_TICKS(SET_MODE_TIMEOUT_MS)) {
                g_panel_mode = 0;
            }
        }

        /* 任务休眠 50ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
