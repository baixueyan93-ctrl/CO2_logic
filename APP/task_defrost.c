#include "task_defrost.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 除霜流程 (逻辑图2) — 实现文件
 *
 * 子逻辑:
 *   1. Defrost_MainProcess()       除霜主程序
 *   2. Defrost_HeatSubroutine()    加热子程序
 *
 * 除霜原理:
 *   蒸发器长时间制冷后表面会结霜, 霜层阻碍换热导致制冷效率下降.
 *   除霜过程: 停止制冷 → 加热蒸发器融霜 → 滴水排出融水 → 恢复制冷
 *
 * 三阶段状态机:
 *   [待机阶段] 正常制冷, 等待除霜间隔到时
 *       ↓ 间隔到时(3小时)
 *   [加热阶段] 开启加热器融霜, 监控时间和温度
 *       ↓ 加热超时(45分钟) 或 蒸发器温度≥10℃
 *   [滴水阶段] 关闭加热器, 等待融水滴干
 *       ↓ 滴水超时(5分钟)
 *   [待机阶段] 除霜完成, 复位间隔计时, 恢复制冷
 * =========================================================================== */


/* ===================================================================
 *  内部辅助函数 / 硬件操作桩
 * =================================================================== */

/* --- 除霜加热器控制 --- */
static void DefrostHeater_On(void)
{
    /* TODO: 确认实际GPIO引脚后填入 */
    HAL_GPIO_WritePin(DEF_HEATER_GPIO_PORT, DEF_HEATER_GPIO_PIN, GPIO_PIN_SET);
}

static void DefrostHeater_Off(void)
{
    HAL_GPIO_WritePin(DEF_HEATER_GPIO_PORT, DEF_HEATER_GPIO_PIN, GPIO_PIN_RESET);
}

/* --- 压缩机停止 (除霜时需要停止制冷) ---
 * 注意: 与温度控制任务共用压缩机, 通过状态标志协调
 */
static void Defrost_StopCompressor(void)
{
    /* TODO: 停止压缩机 (与温度控制任务协调)
     * 除霜期间通过 ST_DEFROST_ACTIVE 标志通知温度控制任务暂停
     */
    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
}

/* --- 加热→滴水 阶段转换 ---
 *
 *  流程图中加热结束后的公共路径:
 *    反标记正在加热(仅做反标记)
 *    → 标记正在滴水
 *    → 开启滴水计时
 *    → 初始化加热时长定时器
 *
 *  "仅做反标记"的含义:
 *    只清除"正在加热"标志, 不清除"正在除霜"总标志,
 *    因为还要进入滴水阶段, 除霜尚未完全结束
 */
static void Defrost_TransitionToDrip(void)
{
    /* 反标记正在加热(仅做反标记) — 关加热器, 清加热标志 */
    DefrostHeater_Off();
    xEventGroupClearBits(SysEventGroup, ST_DEF_HEATING);

    /* 标记正在滴水 */
    xEventGroupSetBits(SysEventGroup, ST_DEF_DRIPPING);

    /* 开启滴水计时 (从0开始, 计时5分钟) */
    g_TimerData.TMR_DRIP_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DRIP_DONE);

    /* 初始化加热时长定时器 (复位, 供下次除霜使用) */
    g_TimerData.TMR_DEF_DUR_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);
}


/* ===================================================================
 *  子逻辑1: 除霜主程序
 *
 *  流程图 (2.除霜流程.pdf 左侧):
 *
 *    开始
 *    → 读取设置值、测量值和定时标记
 *
 *    → 正在除霜?
 *
 *      ┌─ Y(正在除霜) ──────────────────────────────────┐
 *      │                                                 │
 *      │  → 正在滴水(加热不停)?                           │
 *      │                                                 │
 *      │    Y(滴水阶段):                                  │
 *      │      → 滴水时间超时?                             │
 *      │          Y → 反标记正在滴水                       │
 *      │              反标记正在除霜                       │
 *      │              初始化滴水计时、除霜间隔计时          │
 *      │              → 结束 (除霜完全结束, 恢复制冷)      │
 *      │          N → 结束 (继续滴水等待)                  │
 *      │                                                 │
 *      │    N(加热阶段):                                  │
 *      │      → 加热时间超时?                             │
 *      │          Y → [转入滴水]                          │
 *      │          N → 加热温度超值?                        │
 *      │                Y → [转入滴水]                    │
 *      │                N → 结束 (继续加热)               │
 *      │                                                 │
 *      │  [转入滴水]:                                     │
 *      │    反标记正在加热(仅做反标记)                     │
 *      │    → 标记正在滴水                                │
 *      │    → 开启滴水计时                                │
 *      │    → 初始化加热时长定时器                         │
 *      │    → 结束                                        │
 *      │                                                 │
 *      ├─ N(未在除霜) ──────────────────────────────────┐│
 *      │                                                ││
 *      │  → 除霜间隔满时?                                ││
 *      │      Y → 标记正在除霜                           ││
 *      │          标记正在加热                            ││
 *      │          开启加热计时                            ││
 *      │          调用加热子程序                          ││
 *      │          → 结束                                 ││
 *      │      N → (温差条件 + 最短间隔2-3小时, 待扩展)    ││
 *      │          → 结束                                 ││
 *      └─────────────────────────────────────────────────┘│
 *                                                         │
 *    结束                                                  │
 *
 *  调用时机: 每1秒由主循环调用一次
 * =================================================================== */
void Defrost_MainProcess(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

    /* ---- 读取设置值、测量值和定时标记 ----
     * 设置值: SET_DEF_INTERVAL, SET_DEF_HEAT_MAX, SET_DEF_HEAT_TLIMIT,
     *         SET_DRIP_TIME 等 (编译期常量)
     * 测量值: VAR_EVAP_TEMP (蒸发器温度, 判断加热是否达标)
     * 定时标记: ST_TMR_DEF_INTV_DONE, ST_TMR_DEF_DUR_DONE, ST_TMR_DEF_DRIP_DONE
     */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    /* ================================================================
     *  正在除霜?
     * ================================================================ */
    if (sys_bits & ST_DEFROST_ACTIVE) {

        /* ============================================================
         *  Y → 正在除霜中, 判断当前处于哪个阶段
         *
         *  正在滴水(加热不停)?
         *    流程图中"加热不停"的含义:
         *    加热阶段结束后, 加热器已关闭, 但除霜尚未完成,
         *    进入滴水阶段等待融水滴干
         * ============================================================ */

        if (sys_bits & ST_DEF_DRIPPING) {
            /* ========================================================
             *  Y → 当前处于[滴水阶段]
             *
             *  滴水时间超时?
             *    TMR_DRIP_CNT 由定时中断每秒递增,
             *    到达 SET_DRIP_TIME(5分钟) 时置位 ST_TMR_DEF_DRIP_DONE
             * ======================================================== */

            if (tmr_bits & ST_TMR_DEF_DRIP_DONE) {
                /* Y → 滴水完成, 除霜全部结束
                 *
                 * 执行顺序 (严格按流程图):
                 *   1. 反标记正在滴水
                 *   2. 反标记正在除霜
                 *   3. 初始化滴水计时、除霜间隔计时
                 */

                /* 1. 反标记正在滴水 */
                xEventGroupClearBits(SysEventGroup, ST_DEF_DRIPPING);

                /* 2. 反标记正在除霜 — 除霜全部完成 */
                xEventGroupClearBits(SysEventGroup, ST_DEFROST_ACTIVE);

                /* 3. 初始化滴水计时 */
                g_TimerData.TMR_DRIP_CNT = 0;
                xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DRIP_DONE);

                /* 3. 初始化除霜间隔计时 (从0开始, 等待下一次除霜) */
                g_TimerData.TMR_DEF_INTV_CNT = 0;
                xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_INTV_DONE);

                /* 除霜完成后: 延迟15秒后恢复压缩机
                 * SET_DEF_COMP_DLY = 15秒
                 * TODO: 通知温度控制任务可以恢复压缩机
                 *       (温度控制任务通过检查 ST_DEFROST_ACTIVE 已清除来恢复)
                 */
            }
            /* N → 结束: 滴水时间未到, 继续等待 */

        } else {
            /* ========================================================
             *  N → 当前处于[加热阶段]
             *
             *  两个退出条件 (任一满足即转入滴水):
             *    1. 加热时间超时 (SET_DEF_HEAT_MAX = 45分钟)
             *    2. 加热温度超值 (蒸发器温度 ≥ SET_DEF_HEAT_TLIMIT = 10℃)
             * ======================================================== */

            /* ---- 加热时间超时? ---- */
            if (tmr_bits & ST_TMR_DEF_DUR_DONE) {
                /* Y → 加热时间到(45分钟), 无论温度是否达标都转入滴水
                 *     (安全保护: 防止加热器长时间工作)
                 */
                Defrost_TransitionToDrip();
                return;
            }

            /* ---- N → 加热温度超值? ---- */
            if (sensor.VAR_EVAP_TEMP >= SET_DEF_HEAT_TLIMIT) {
                /* Y → 蒸发器温度已达标(≥10℃), 霜已融化, 提前转入滴水
                 *     (正常情况: 温度到了就不需要继续加热)
                 */
                Defrost_TransitionToDrip();
                return;
            }

            /* N → 结束: 继续加热 (时间未到, 温度未达标) */
        }

    } else {
        /* ============================================================
         *  N → 未在除霜, 当前处于[待机阶段]
         *
         *  除霜间隔满时?
         *    TMR_DEF_INTV_CNT 由定时中断每秒递增,
         *    到达 SET_DEF_INTERVAL(3小时) 时置位 ST_TMR_DEF_INTV_DONE
         * ============================================================ */

        if (tmr_bits & ST_TMR_DEF_INTV_DONE) {
            /* Y → 除霜间隔到时, 开始除霜
             *
             * 执行顺序 (严格按流程图):
             *   1. 标记正在除霜
             *   2. 标记正在加热
             *   3. 开启加热计时
             *   4. 调用加热子程序
             */

            /* 1. 标记正在除霜 — 进入除霜总状态 */
            xEventGroupSetBits(SysEventGroup, ST_DEFROST_ACTIVE);

            /* 通知温控任务: 除霜期间需停止压缩机
             * (温度控制任务检测到 ST_DEFROST_ACTIVE 后应暂停制冷)
             */
            Defrost_StopCompressor();

            /* 2. 标记正在加热 — 进入加热阶段 */
            xEventGroupSetBits(SysEventGroup, ST_DEF_HEATING);

            /* 3. 开启加热计时 (从0开始, 计时45分钟) */
            g_TimerData.TMR_DEF_DUR_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);

            /* 开启加热器 */
            DefrostHeater_On();

            /* 4. 调用加热子程序
             * (处理膨胀阀步进 + 延时后开压缩机等启动序列)
             */
            Defrost_HeatSubroutine();
        }

        /* N → 结束: 除霜间隔未到, 继续等待
         *
         * TODO: 温差条件提前触发除霜
         *   流程图底部标注: "温差" + "最短间隔时间 2-3小时"
         *   含义: 即使间隔未到3小时, 如果检测到蒸发器与柜温温差异常大,
         *         且已过最短间隔(2小时), 也可提前触发除霜
         *
         *   实现思路 (待完善):
         *     float temp_diff = sensor.VAR_CABINET_TEMP - sensor.VAR_EVAP_TEMP;
         *     if (temp_diff > 某阈值 &&
         *         g_TimerData.TMR_DEF_INTV_CNT >= SET_DEF_MIN_INTV) {
         *         // 提前触发除霜
         *     }
         */
    }
}


/* ===================================================================
 *  子逻辑2: 加热子程序 (框架桩)
 *
 *  流程图 (2.除霜流程.pdf 右侧):
 *
 *    加热子程序
 *    → 延时X时间够?
 *        N → 结束 (等待下次调用)
 *        Y → 延时步进 (膨胀阀动作)
 *    → 延时X时间够?
 *        N → 结束
 *        Y → 15s后开压缩机, 由慢到快步进
 *    → 测温度
 *    → 结束
 *
 *  说明: 除霜启动时的硬件初始化序列
 *        控制膨胀阀步进到位, 延时后启动压缩机热气除霜
 *        SET_DEF_STEP_DLY = 30秒 (延时X)
 *        SET_DEF_COMP_DLY = 15秒 (压缩机延迟)
 *
 *  TODO: 第二步实现此子程序
 * =================================================================== */
void Defrost_HeatSubroutine(void)
{
    /* TODO: 实现加热子程序
     *
     * 基本框架:
     *   1. 控制膨胀阀步进到除霜位置 (通过 task_exv 接口)
     *   2. 等待延时X (SET_DEF_STEP_DLY = 30秒)
     *   3. 延时后开压缩机, 由慢到快步进频率
     *   4. 等待15秒 (SET_DEF_COMP_DLY)
     *   5. 测温度确认加热开始
     *
     * 注意: 此函数不能阻塞 (在FreeRTOS任务中),
     *       需要用状态机方式分步执行,
     *       或由定时标志驱动各步骤
     */
}


/* ===================================================================
 *  除霜任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒执行一轮:
 *    1. 除霜主程序 (状态机: 待机→加热→滴水→待机)
 *
 *  循环周期: 1秒
 * =================================================================== */
void Task_Defrost_Process(void const *argument)
{
    (void)argument;

    /* 初始化: 启动除霜间隔计时
     * 上电后开始计时, 到达3小时后执行第一次除霜
     */
    g_TimerData.TMR_DEF_INTV_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_INTV_DONE);
    g_TimerData.TMR_DEF_DUR_CNT = 0;
    g_TimerData.TMR_DRIP_CNT = 0;

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        /* ============================================
         * 除霜主程序 (每秒执行)
         *   状态机驱动: 根据当前阶段执行对应逻辑
         * ============================================ */
        Defrost_MainProcess();

        /* 循环延时: 1秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
