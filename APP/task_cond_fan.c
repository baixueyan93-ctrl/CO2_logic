#include "task_cond_fan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 冷凝风机(3台)流程 (逻辑图6) — 实现文件
 *
 * 子逻辑:
 *   1. CondFan_MainProcess()   冷凝风机主逻辑
 *
 * 冷凝器原理:
 *   压缩机排出高温高压制冷剂气体, 经冷凝器散热后液化.
 *   冷凝器散热依靠风机吹风, 风机越多散热越快.
 *   冷凝温度过高 → 系统效率下降, 高压报警风险.
 *
 * 控制策略 (温度逻辑):
 *   根据冷凝温度 VAR_COND_TEMP 决定投入风机数量:
 *
 *   温度区间                      风机投入
 *   ─────────────────────────────────────
 *   ≥ SET_COND_FAN_ON_T (35℃)    温度较高, 需要多台风机散热
 *   ≤ SET_COND_FAN_OFF_T (25℃)   温度较低, 可以减少风机
 *
 *   阶梯投入 (3台):
 *     冷凝温度 ≥ 35℃                → 3台全开
 *     冷凝温度 在 30~35℃            → 2台开
 *     冷凝温度 在 25~30℃            → 1台开
 *     冷凝温度 < 25℃                → 全部关闭
 *
 *   温差阶梯 = (SET_COND_FAN_ON_T - SET_COND_FAN_OFF_T) / 2 = 5℃
 *   第1台开启: 25℃   第2台开启: 30℃   第3台开启: 35℃
 *
 * 当前控制方式: 继电器开关 (现有控制: 开关)
 * 未来扩展: EC风机485变频 / DC风机PWM (流程图标注)
 * =========================================================================== */


/* ===================================================================
 *  内部辅助函数 / 硬件操作桩
 * =================================================================== */

/* --- 冷凝风机1 开/关 --- */
static void CondFan1_On(void)
{
    HAL_GPIO_WritePin(COND_FAN1_GPIO_PORT, COND_FAN1_GPIO_PIN, GPIO_PIN_SET);
    xEventGroupSetBits(SysEventGroup, ST_COND_FAN1_ON);
}

static void CondFan1_Off(void)
{
    HAL_GPIO_WritePin(COND_FAN1_GPIO_PORT, COND_FAN1_GPIO_PIN, GPIO_PIN_RESET);
    xEventGroupClearBits(SysEventGroup, ST_COND_FAN1_ON);
}

/* --- 冷凝风机2 开/关 --- */
static void CondFan2_On(void)
{
    HAL_GPIO_WritePin(COND_FAN2_GPIO_PORT, COND_FAN2_GPIO_PIN, GPIO_PIN_SET);
    xEventGroupSetBits(SysEventGroup, ST_COND_FAN2_ON);
}

static void CondFan2_Off(void)
{
    HAL_GPIO_WritePin(COND_FAN2_GPIO_PORT, COND_FAN2_GPIO_PIN, GPIO_PIN_RESET);
    xEventGroupClearBits(SysEventGroup, ST_COND_FAN2_ON);
}

/* --- 冷凝风机3 开/关 --- */
static void CondFan3_On(void)
{
    HAL_GPIO_WritePin(COND_FAN3_GPIO_PORT, COND_FAN3_GPIO_PIN, GPIO_PIN_SET);
    xEventGroupSetBits(SysEventGroup, ST_COND_FAN3_ON);
}

static void CondFan3_Off(void)
{
    HAL_GPIO_WritePin(COND_FAN3_GPIO_PORT, COND_FAN3_GPIO_PIN, GPIO_PIN_RESET);
    xEventGroupClearBits(SysEventGroup, ST_COND_FAN3_ON);
}

/* --- 关闭所有冷凝风机 --- */
static void CondFan_AllOff(void)
{
    CondFan1_Off();
    CondFan2_Off();
    CondFan3_Off();
}


/* ===================================================================
 *  子逻辑1: 冷凝风机主逻辑
 *
 *  流程图 (6.冷凝风机流程.pdf):
 *
 *    开始
 *    → 读取设置值、测量值和定时标记
 *
 *    → 压缩机运行?
 *
 *      ┌─ N(压缩机未运行) ─────────────────────┐
 *      │  → 风机正在运行?                       │
 *      │    Y → 关闭风机 → 结束                 │
 *      │    N → 结束                            │
 *      │                                        │
 *      ├─ Y(压缩机运行) ──────────────────────┐ │
 *      │  → 风机正在运行?                      │ │
 *      │                                       │ │
 *      │    Y → 时间逻辑? (红色, TODO)         │ │
 *      │        → 结束 (未来扩展)              │ │
 *      │                                       │ │
 *      │    N → 温度逻辑                       │ │
 *      │        根据冷凝温度确定风机编号        │ │
 *      │        ┌──────────────────────┐       │ │
 *      │        │ 1: 确定风机编号(1台) │       │ │
 *      │        │ 2: 确定风机编号(2台) │       │ │
 *      │        │ 3: 确定风机编号(3台) │       │ │
 *      │        └──────┬───────────────┘       │ │
 *      │               ▼                       │ │
 *      │        开启风机                        │ │
 *      │        标记风机正在运行                 │ │
 *      │        → 结束                          │ │
 *      └────────────────────────────────────────┘ │
 *
 *  温度逻辑 — 阶梯投入:
 *    温差阶梯 = (ON温度 - OFF温度) / 2 = (35-25)/2 = 5℃
 *    T_step1 = OFF温度          = 25℃  → 开1台
 *    T_step2 = OFF温度 + 阶梯   = 30℃  → 开2台
 *    T_step3 = ON温度           = 35℃  → 开3台
 *    < OFF温度                          → 全关
 *
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void CondFan_MainProcess(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    bool comp_on = (sys_bits & ST_COMP_RUNNING) != 0;
    bool any_fan_on = (sys_bits & (ST_COND_FAN1_ON |
                                    ST_COND_FAN2_ON |
                                    ST_COND_FAN3_ON)) != 0;

    /* ================================================================
     *  第1步: 压缩机运行?
     * ================================================================ */
    if (!comp_on) {
        /* ============================================================
         *  N → 压缩机未运行
         *
         *  风机正在运行?
         *    Y → 关闭所有风机
         *    N → 结束 (已经关着)
         *
         *  压缩机停了就不需要冷凝散热
         * ============================================================ */
        if (any_fan_on) {
            CondFan_AllOff();
        }
        return;
    }

    /* ================================================================
     *  Y → 压缩机运行中
     *
     *  第2步: 风机正在运行?
     * ================================================================ */
    if (any_fan_on) {
        /* ============================================================
         *  Y → 风机已经在运行
         *
         *  时间逻辑? (红色标注, 未来扩展)
         *
         *  TODO: 未来可根据运行时间调整风机台数
         *        例如: 压缩机运行超过X分钟后逐步增加风机
         *        或: 根据冷凝温度变化趋势动态调整
         *
         *  当前实现: 已在运行时也按温度逻辑重新评估台数
         *           (这样温度变化时可以动态增减风机)
         * ============================================================ */
        /* 继续往下执行温度逻辑, 动态调整风机台数 */
    }

    /* ================================================================
     *  第3步: 温度逻辑 — 根据冷凝温度确定风机编号
     *
     *  读取冷凝器温度, 按阶梯决定投入几台风机
     *
     *  阶梯计算:
     *    ON温度  = SET_COND_FAN_ON_T  = 35℃
     *    OFF温度 = SET_COND_FAN_OFF_T = 25℃
     *    阶梯宽度 = (35 - 25) / 2 = 5℃
     *
     *    阈值:
     *      T ≥ 35℃ (ON温度)          → 3台全开
     *      T ≥ 30℃ (OFF温度+阶梯)    → 2台开
     *      T ≥ 25℃ (OFF温度)         → 1台开
     *      T <  25℃                  → 全部关闭
     * ================================================================ */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    float cond_temp = sensor.VAR_COND_TEMP;

    /* 阶梯阈值计算 */
    float t_step = (SET_COND_FAN_ON_T - SET_COND_FAN_OFF_T) / 2.0f;  /* 5℃ */
    float t_fan1 = SET_COND_FAN_OFF_T;                 /* 25℃: 开第1台 */
    float t_fan2 = SET_COND_FAN_OFF_T + t_step;        /* 30℃: 开第2台 */
    float t_fan3 = SET_COND_FAN_ON_T;                  /* 35℃: 开第3台 */

    /* 确定需要投入的风机台数 */
    uint8_t fans_needed;

    if (cond_temp >= t_fan3) {
        fans_needed = 3;    /* ≥ 35℃: 3台全开 (流程图编号3) */
    } else if (cond_temp >= t_fan2) {
        fans_needed = 2;    /* 30~35℃: 2台开 (流程图编号2) */
    } else if (cond_temp >= t_fan1) {
        fans_needed = 1;    /* 25~30℃: 1台开 (流程图编号1) */
    } else {
        fans_needed = 0;    /* < 25℃: 全部关闭 */
    }

    /* ================================================================
     *  第4步: 开启/关闭风机 + 标记风机正在运行
     *
     *  流程图: 确定风机编号 → 开启风机 → 标记风机正在运行
     *
     *  按需投入: 需要几台开几台, 多余的关掉
     *  顺序: 先开1→2→3, 先关3→2→1
     * ================================================================ */
    switch (fans_needed) {
    case 3:
        CondFan1_On();
        CondFan2_On();
        CondFan3_On();
        break;

    case 2:
        CondFan1_On();
        CondFan2_On();
        CondFan3_Off();
        break;

    case 1:
        CondFan1_On();
        CondFan2_Off();
        CondFan3_Off();
        break;

    default:  /* 0: 全部关闭 */
        CondFan_AllOff();
        break;
    }
}


/* ===================================================================
 *  冷凝风机任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒执行一轮:
 *    1. 冷凝风机主逻辑 (压缩机状态+冷凝温度→风机台数)
 *
 *  循环周期: 1秒
 * =================================================================== */
void Task_CondFan_Process(void const *argument)
{
    (void)argument;

    /* 初始化: 所有冷凝风机默认关闭 */
    CondFan_AllOff();

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        /* ============================================
         * 冷凝风机主逻辑 (每秒执行)
         *   根据压缩机状态和冷凝温度控制风机
         * ============================================ */
        CondFan_MainProcess();

        /* 循环延时: 1秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
