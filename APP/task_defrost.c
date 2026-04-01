#include "task_defrost.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_relay.h"
#include "bsp_exv.h"
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

/* --- 除霜加热器控制 (通过 K7 继电器, PC7) --- */
static void DefrostHeater_On(void)
{
    BSP_Relay_On(RELAY_DEF_HEATER);
}

static void DefrostHeater_Off(void)
{
    BSP_Relay_Off(RELAY_DEF_HEATER);
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

/* --- 除霜时启动压缩机 (热气除霜) ---
 * 与正常制冷不同: 热气除霜用压缩机排出的高温气体加热蒸发器
 */
static void Defrost_StartCompressor(float freq_hz)
{
    /* TODO: 通过变频器通信启动压缩机并设置频率
     * 除霜模式下的压缩机运行方式可能与制冷模式不同
     */
    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);
    SysState_Lock();
    SysState_GetRawPtr()->VAR_COMP_FREQ = freq_hz;
    SysState_Unlock();
}

/* --- 膨胀阀步进到除霜位置 --- */
#define DEF_EXV_POSITION   1000  /* 除霜时全开 (让高温制冷剂通过蒸发器) */

static void Defrost_ValveStep(void)
{
    BSP_EXV_SetPosition(DEF_EXV_POSITION, EXV_STEP_DELAY_MS);
    BSP_EXV_DeEnergize();
}

/* ===================================================================
 *  加热子程序状态机定义
 *
 *  流程图中加热子程序包含延时和多步操作,
 *  不能阻塞FreeRTOS任务, 所以用状态机实现,
 *  每秒被主循环调用一次, 逐步推进
 * =================================================================== */
typedef enum {
    HEAT_IDLE,           /* 空闲, 等待启动                           */
    HEAT_WAIT_DELAY1,    /* 等待第一个延时X (SET_DEF_STEP_DLY = 30s) */
    HEAT_VALVE_STEP,     /* 执行膨胀阀步进动作                       */
    HEAT_WAIT_DELAY2,    /* 等待第二个延时X (30s)                    */
    HEAT_COMP_DELAY,     /* 等待15s后开压缩机 (SET_DEF_COMP_DLY)    */
    HEAT_COMP_RAMP,      /* 压缩机由慢到快步进升频                   */
    HEAT_RUNNING         /* 加热运行中, 测温度                       */
} DefrostHeatState_t;

static DefrostHeatState_t s_heat_state = HEAT_IDLE;
static uint32_t s_heat_delay_cnt = 0;     /* 加热子程序内部延时计数器 */
static float    s_comp_ramp_freq = 0.0f;  /* 压缩机升频当前值       */
static bool     s_was_defrosting = false; /* 上一周期是否在除霜 (检测手动取消) */

/* --- 加热子程序状态机复位 --- */
static void Defrost_HeatReset(void)
{
    s_heat_state = HEAT_IDLE;
    s_heat_delay_cnt = 0;
    s_comp_ramp_freq = 0.0f;
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

    /* 停止除霜用的压缩机 (滴水阶段不需要压缩机运行) */
    Defrost_StopCompressor();

    /* 复位加热子程序状态机 */
    Defrost_HeatReset();

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

    /* ---- 读取设置值、测量值和定时标记 ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    /* ================================================================
     *  手动取消检测: 上周期在除霜, 本周期 ST_DEFROST_ACTIVE 被外部清除
     *  面板按键再按一次除霜键时清除所有除霜标志, 这里负责安全收尾
     * ================================================================ */
    if (s_was_defrosting && !(sys_bits & ST_DEFROST_ACTIVE)) {
        /* 关闭加热器 + 停压缩机 + 复位状态机 */
        DefrostHeater_Off();
        Defrost_StopCompressor();
        Defrost_HeatReset();

        /* 复位计时器, 重新开始除霜间隔计时 */
        g_TimerData.TMR_DEF_INTV_CNT = 0;
        xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_INTV_DONE);
        g_TimerData.TMR_DEF_DUR_CNT = 0;
        g_TimerData.TMR_DRIP_CNT = 0;

        s_was_defrosting = false;
        return;
    }
    s_was_defrosting = !!(sys_bits & ST_DEFROST_ACTIVE);

    /* ================================================================
     *  正在除霜?
     * ================================================================ */
    if (sys_bits & ST_DEFROST_ACTIVE) {

        /* ============================================================
         *  Y → 正在除霜中, 判断当前处于哪个阶段
         * ============================================================ */

        /* ---- 手动触发检测 ----
         * 如果 ST_DEFROST_ACTIVE 已置位, 但既不在加热也不在滴水,
         * 说明是由面板手动除霜键触发 (只置位了总标志).
         * 此时需要执行完整的除霜启动序列.
         */
        if (!(sys_bits & ST_DEF_HEATING) && !(sys_bits & ST_DEF_DRIPPING)) {
            /* 手动触发 → 执行与定时触发相同的启动序列 */
            Defrost_StopCompressor();

            xEventGroupSetBits(SysEventGroup, ST_DEF_HEATING);

            g_TimerData.TMR_DEF_DUR_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_DEF_DUR_DONE);

            DefrostHeater_On();
            Defrost_HeatReset();
            Defrost_HeatSubroutine();
            return;
        }

        /* ============================================================
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

                /* 复位加热子程序状态机 (安全保障) */
                Defrost_HeatReset();

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

            /* 每周期驱动加热子程序状态机 (膨胀阀步进 + 压缩机启动) */
            Defrost_HeatSubroutine();

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
 *  子逻辑2: 加热子程序
 *
 *  流程图 (2.除霜流程.pdf 右侧):
 *
 *    加热子程序
 *    → 延时X时间够?             ← 第一个延时(30s): 等待系统稳定
 *        N → 结束 (等待下次调用)
 *        Y → 延时步进             ← 膨胀阀步进到除霜位置
 *    → 延时X时间够?             ← 第二个延时(30s): 等待阀门到位
 *        N → 结束
 *        Y → 15s后开压缩机       ← 等15s后启动压缩机
 *            由慢到快步进         ← 压缩机频率逐步升高
 *    → 测温度                    ← 确认加热已开始工作
 *    → 结束
 *
 *  实现方式: 状态机 (不阻塞, 每秒被主循环调用一次)
 *
 *  状态转换:
 *    IDLE → WAIT_DELAY1(30s) → VALVE_STEP → WAIT_DELAY2(30s)
 *         → COMP_DELAY(15s) → COMP_RAMP(升频) → RUNNING(测温)
 *
 *  延时X = SET_DEF_STEP_DLY (30秒)
 *  压缩机延迟 = SET_DEF_COMP_DLY (15秒)
 * =================================================================== */
void Defrost_HeatSubroutine(void)
{
    switch (s_heat_state) {

    /* ================================================================
     *  HEAT_IDLE: 空闲 → 启动第一个延时X
     *
     *  刚进入加热阶段, 初始化延时计数器,
     *  进入第一个等待周期
     * ================================================================ */
    case HEAT_IDLE:
        s_heat_delay_cnt = 0;
        s_heat_state = HEAT_WAIT_DELAY1;
        break;

    /* ================================================================
     *  HEAT_WAIT_DELAY1: 延时X时间够? (第一个30s)
     *
     *  流程图: "延时X时间够? → N → 结束"
     *  等待30秒让系统稳定 (停机后管路压力平衡)
     * ================================================================ */
    case HEAT_WAIT_DELAY1:
        s_heat_delay_cnt++;
        if (s_heat_delay_cnt >= SET_DEF_STEP_DLY) {
            /* Y → 延时到, 进入膨胀阀步进 */
            s_heat_state = HEAT_VALVE_STEP;
        }
        /* N → 结束(等待下次调用) */
        break;

    /* ================================================================
     *  HEAT_VALVE_STEP: 延时步进 (膨胀阀动作)
     *
     *  流程图: "延时步进"
     *  将膨胀阀步进到除霜位置, 让高温制冷剂能通过蒸发器
     *  动作完成后立即进入第二个延时等待
     * ================================================================ */
    case HEAT_VALVE_STEP:
        Defrost_ValveStep();           /* 膨胀阀步进到除霜位置 */
        s_heat_delay_cnt = 0;          /* 复位计数, 准备第二个延时 */
        s_heat_state = HEAT_WAIT_DELAY2;
        break;

    /* ================================================================
     *  HEAT_WAIT_DELAY2: 延时X时间够? (第二个30s)
     *
     *  流程图: 第二个"延时X时间够? → N → 结束"
     *  等待30秒让膨胀阀完全到位
     * ================================================================ */
    case HEAT_WAIT_DELAY2:
        s_heat_delay_cnt++;
        if (s_heat_delay_cnt >= SET_DEF_STEP_DLY) {
            /* Y → 延时到, 准备开压缩机 */
            s_heat_delay_cnt = 0;
            s_heat_state = HEAT_COMP_DELAY;
        }
        /* N → 结束(等待下次调用) */
        break;

    /* ================================================================
     *  HEAT_COMP_DELAY: 15s后开压缩机
     *
     *  流程图: "15s后开压缩机"
     *  再等15秒 (SET_DEF_COMP_DLY), 然后启动压缩机
     * ================================================================ */
    case HEAT_COMP_DELAY:
        s_heat_delay_cnt++;
        if (s_heat_delay_cnt >= SET_DEF_COMP_DLY) {
            /* 15s到 → 启动压缩机, 初始低频 */
            s_comp_ramp_freq = SET_FREQ_MIN;   /* 从最低频率开始 (20Hz) */
            Defrost_StartCompressor(s_comp_ramp_freq);
            s_heat_delay_cnt = 0;
            s_heat_state = HEAT_COMP_RAMP;
        }
        /* N → 结束(等待15s) */
        break;

    /* ================================================================
     *  HEAT_COMP_RAMP: 由慢到快步进
     *
     *  流程图: "由慢到快步进"
     *  压缩机频率从最低(20Hz)逐步升到目标频率(125Hz)
     *  每秒增加一定步长, 避免瞬间高负荷
     * ================================================================ */
    case HEAT_COMP_RAMP: {
        /* 每秒升频, 直到达到目标频率 */
        const float RAMP_STEP = 10.0f;     /* 每秒升10Hz (待确认) */
        const float RAMP_TARGET = SET_FREQ_INIT;  /* 目标: 125Hz */

        s_comp_ramp_freq += RAMP_STEP;
        if (s_comp_ramp_freq >= RAMP_TARGET) {
            s_comp_ramp_freq = RAMP_TARGET;
            s_heat_state = HEAT_RUNNING;   /* 升频完成, 进入运行状态 */
        }

        /* 更新压缩机频率 */
        SysState_Lock();
        SysState_GetRawPtr()->VAR_COMP_FREQ = s_comp_ramp_freq;
        SysState_Unlock();

        /* TODO: 通过变频器通信实际设置频率 */
        break;
    }

    /* ================================================================
     *  HEAT_RUNNING: 测温度 → 加热运行中
     *
     *  流程图: "测温度 → 结束"
     *  压缩机已达到目标频率, 加热正常运行,
     *  温度监控和退出判断由主程序 Defrost_MainProcess() 负责:
     *    - 加热时间超时(45min) → 转入滴水
     *    - 蒸发器温度≥10℃    → 转入滴水
     *
     *  此状态下子程序无需额外操作, 仅保持运行
     * ================================================================ */
    case HEAT_RUNNING:
        /* 加热运行中, 无需操作 (主程序监控退出条件) */
        break;

    default:
        /* 异常状态, 复位 */
        Defrost_HeatReset();
        break;
    }
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
