#include "task_freq_exv.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include "bsp_exv.h"
#include <stdbool.h>
#include <math.h>

/* ===========================================================================
 * 变频控制(PID)和膨胀阀流程 (逻辑图4) — 实现文件
 *
 * 子逻辑:
 *   1. FreqExv_PidAdjust()     PID调整模块 — 压缩机频率调节
 *   2. FreqExv_ExvAdjust()     膨胀阀调整模块 — 膨胀阀开度(Kp)调节
 *
 * 两个模块每30秒同步执行一次 (PID周期 = SET_PID_PERIOD = 30s)
 *
 * 变频控制原理:
 *   CO2制冷系统通过调节压缩机运转频率来控制制冷量.
 *   柜温高于设定 → 升频加大制冷; 柜温低于设定 → 降频减小制冷.
 *   目标是让柜温稳定在设定温度 ± C1回差带范围内.
 *
 * 膨胀阀原理:
 *   电子膨胀阀(EXV)控制制冷剂流量, 影响蒸发器换热效率.
 *   通过传热温差(△TCZ)和过热度(△TP)两个指标来调节阀门开度(Kp).
 *   传热温差反映蒸发器换热效率, 过热度保护压缩机安全.
 * =========================================================================== */


/* ===================================================================
 *  内部常量定义
 * =================================================================== */

/* --- PID调整模块内部参数 --- */
#define PID_FREQ_STEP_MAX       20.0f   /* 最大频率调整步长 (Hz/次), 待调参 */
#define PID_FREQ_MAX            125.0f  /* 压缩机最大频率 (Hz) = SET_FREQ_INIT */

/* --- 膨胀阀调整模块内部参数 --- */
#define EXV_STEP_SIZE           10      /* 每次调整步进10步 (全行程1000步)  */


/* ===================================================================
 *  内部状态变量 (静态, 跨调用保持)
 * =================================================================== */

/* --- PID模块: 上一次△T, 用于判断"△T缩小" --- */
static float s_prev_delta_t = 0.0f;
static bool  s_prev_dt_valid = false;   /* 首次运行时无上一次数据 */


/* ===================================================================
 *  内部辅助函数
 * =================================================================== */

/* --- 设置压缩机频率 (含上下限保护) --- */
static void PID_SetFreq(float freq_hz)
{
    /* 上限保护 */
    if (freq_hz > PID_FREQ_MAX) {
        freq_hz = PID_FREQ_MAX;
    }
    /* 下限保护 */
    if (freq_hz < SET_FREQ_MIN) {
        freq_hz = SET_FREQ_MIN;
    }

    /* 写入全局传感器数据 (变频器实际设置由通信任务读取此值) */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_COMP_FREQ = freq_hz;
    SysState_Unlock();

    /* TODO: 通过变频器通信接口实际设置频率 */
}

/* --- 设置膨胀阀开度 (含上下限保护) --- */
static void EXV_SetOpening(float kp)
{
    /* 下限保护: 开度不能为负 */
    if (kp < 0.0f) {
        kp = 0.0f;
    }
    /* 上限保护: VKV 阀体全行程 1000 步 */
    if (kp > (float)EXV_TOTAL_STEPS) {
        kp = (float)EXV_TOTAL_STEPS;
    }

    /* 写入全局传感器数据 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_EXV_OPENING = kp;
    SysState_Unlock();

    /* 驱动步进电机到目标位置, 完成后断电省功耗 */
    BSP_EXV_SetPosition((uint16_t)(kp + 0.5f), EXV_STEP_DELAY_MS);
    BSP_EXV_DeEnergize();
}

/* --- 压缩机停机 (ETM错误时调用) --- */
static void PID_StopCompressor(void)
{
    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
    /* TODO: 发送变频器停止指令 */
}


/* ===================================================================
 *  子逻辑1: PID调整模块 (左侧)
 *
 *  流程图 (4.变频控制和膨胀阀流程.pdf 左侧):
 *
 *    开始
 *    → 计算 △T = T1 - Ts
 *
 *    → △T ≥ △Tmax?
 *        是 → 最快速升频 → 结束
 *
 *    → △Tmax > △T ≥ △Tmin?
 *        是 → 比例升频 → 结束
 *
 *    → |△T| ≤ △Tmin (在C1回差带内)?
 *        是 → 结束 (维持当前频率, 不调整)
 *
 *    → 否 (△T < -△Tmin, 柜温低于设定):
 *        比例降频
 *        → 记录△T
 *        → △T缩小?
 *            是 → 结束 (趋势良好, 等下次周期)
 *            否 → F ≤ Fmin?
 *                否 → 结束 (频率还没到底, 继续下次降)
 *                是 → ETM报警 → 停机 → 结束
 *
 *  各区间含义:
 *    △T ≥ △Tmax (5℃):  柜温远高于设定, 需要最大制冷力
 *    △Tmin~△Tmax (1~5℃): 柜温偏高, 按偏差比例升频
 *    |△T| ≤ △Tmin (±1℃): 已接近目标, 维持不动 (C1回差带)
 *    △T < -△Tmin:         柜温偏低, 按偏差比例降频
 *    降频后△T不缩小+F到底: 异常, 可能制冷系统故障, 停机保护
 *
 *  调用时机: 每30秒由主循环调用一次
 * =================================================================== */
void FreqExv_PidAdjust(void)
{
    /* ---- 读取传感器数据 ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    /* 只在压缩机运行时才调整频率 */
    if (!(sys_bits & ST_COMP_RUNNING)) {
        return;
    }

    /* ================================================================
     *  第1步: 计算 △T = T1 - Ts
     *
     *  T1 = VAR_CABINET_TEMP (柜温, 实测)
     *  Ts = SET_TEMP_TS (设定目标温度, -20℃)
     *  △T > 0: 柜温高于设定, 需要加强制冷 (升频)
     *  △T < 0: 柜温低于设定, 需要减弱制冷 (降频)
     *  △T ≈ 0: 已达目标, 维持
     * ================================================================ */
    float delta_t = sensor.VAR_CABINET_TEMP - SET_TEMP_TS;
    float current_freq = sensor.VAR_COMP_FREQ;
    float new_freq = current_freq;

    /* 同步更新 VAR_DELTA_T 供其他模块使用 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_DELTA_T = delta_t;
    SysState_Unlock();

    /* ================================================================
     *  第2步: △T ≥ △Tmax?
     *
     *  △Tmax = SET_DT_MAX = 5℃
     *  柜温比设定高5℃以上 → 最快速升频
     *  直接设为最大频率, 全力制冷
     * ================================================================ */
    if (delta_t >= SET_DT_MAX) {
        /* 是 → 最快速升频 */
        new_freq = PID_FREQ_MAX;
        PID_SetFreq(new_freq);
        return;
    }

    /* ================================================================
     *  第3步: △Tmax > △T ≥ △Tmin?
     *
     *  △Tmin = SET_DT_MIN = 1℃
     *  柜温偏高1~5℃ → 比例升频
     *  偏差越大升频越多, 线性插值:
     *    step = PID_FREQ_STEP_MAX × (△T - △Tmin) / (△Tmax - △Tmin)
     * ================================================================ */
    if (delta_t >= SET_DT_MIN) {
        /* 是 → 比例升频 */
        float ratio = (delta_t - SET_DT_MIN) / (SET_DT_MAX - SET_DT_MIN);
        float step = PID_FREQ_STEP_MAX * ratio;
        /* 最少升1Hz, 避免零调整 */
        if (step < 1.0f) {
            step = 1.0f;
        }
        new_freq = current_freq + step;
        PID_SetFreq(new_freq);
        return;
    }

    /* ================================================================
     *  第4步: |△T| ≤ △Tmin 不是C1?
     *
     *  即 |△T| ≤ 1℃ (C1回差带范围内)
     *  柜温已非常接近设定温度 → 维持当前频率, 不调整
     * ================================================================ */
    if (fabsf(delta_t) <= SET_TEMP_HYST_C1) {
        /* 是 → 结束 (在C1回差带内, 维持) */
        return;
    }

    /* ================================================================
     *  第5步: 比例降频
     *
     *  执行到这里说明: △T < -C1, 柜温明显低于设定
     *  需要降低压缩机频率减少制冷量
     *
     *  降频幅度与偏差成正比:
     *    step = PID_FREQ_STEP_MAX × |△T| / △Tmax
     * ================================================================ */
    {
        float abs_dt = fabsf(delta_t);
        float ratio = abs_dt / SET_DT_MAX;
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        float step = PID_FREQ_STEP_MAX * ratio;
        if (step < 1.0f) {
            step = 1.0f;
        }
        new_freq = current_freq - step;

        /* 频率下限保护 (在PID_SetFreq中也有, 这里提前处理便于后续判断) */
        if (new_freq < SET_FREQ_MIN) {
            new_freq = SET_FREQ_MIN;
        }
        PID_SetFreq(new_freq);
    }

    /* ================================================================
     *  第6步: 记录△T
     *
     *  保存当前△T, 供下一个PID周期比较
     * ================================================================ */
    float prev_dt = s_prev_delta_t;
    bool  prev_valid = s_prev_dt_valid;

    s_prev_delta_t = delta_t;
    s_prev_dt_valid = true;

    /* ================================================================
     *  第7步: △T缩小?
     *
     *  比较当前|△T|与上次|△T|:
     *    |当前△T| < |上次△T| → 缩小 (趋势在改善)
     *    否则 → 没缩小 (趋势恶化或停滞)
     *
     *  首次运行无上次数据, 默认视为"缩小" (给系统一次调整机会)
     * ================================================================ */
    if (!prev_valid || fabsf(delta_t) < fabsf(prev_dt)) {
        /* 是 → △T在缩小(趋势良好), 标记并结束 */
        xEventGroupSetBits(SysEventGroup, ST_DT_SHRINKING);
        return;
    }

    /* ---- 否 → △T未缩小 ---- */
    xEventGroupClearBits(SysEventGroup, ST_DT_SHRINKING);

    /* ================================================================
     *  第8步: F ≤ Fmin?
     *
     *  频率已降至最低(20Hz), 且温度偏差没有缩小
     *  说明制冷系统可能有问题 (制冷量无法减小到匹配柜温)
     * ================================================================ */
    if (new_freq <= SET_FREQ_MIN) {
        /* ---- 是 → ETM报警 → 停机 ----
         *
         *  ETM = ERR_TEMP_LOW_STOP: 频率到底温度异常停机
         *  这是严重错误, 需要停机保护
         */
        g_AlarmFlags |= ERR_TEMP_LOW_STOP;

        /* 停机 */
        PID_StopCompressor();

        /* TODO: 通知用户 (面板显示ETM错误码) */
    }
    /* 否 → 频率还没到底, 下次周期继续降频, 结束 */
}


/* ===================================================================
 *  子逻辑2: 膨胀阀调整模块 (右侧)
 *
 *  流程图 (4.变频控制和膨胀阀流程.pdf 右侧):
 *
 *    开始
 *    → 传热温差 △TCZ = 柜温 - 蒸发温度
 *       △TCZ ≥ 6.5±1.5 (即 △TCZ 在 5.0~8.0 范围内)?
 *
 *      ┌─ N(不在范围) ─────────────────────────────┐
 *      │                                            │
 *      │  → Kp = Kp - α1·△TCZ                     │
 *      │    (传热温差偏离目标, 按α1系数调整阀门开度) │
 *      │  → 输出Kp → 结束                          │
 *      │                                            │
 *      ├─ Y(在范围内) ─────────────────────────────┐│
 *      │                                           ││
 *      │  → 过热度 △TP < △TPmin (6.5~8)?          ││
 *      │                                           ││
 *      │    ┌─ 是 (过热度过低) ──────────┐         ││
 *      │    │  → EDT 报警                │         ││
 *      │    │    (过热度低, 液态制冷剂    │         ││
 *      │    │     可能进入压缩机, 危险)   │         ││
 *      │    │  → Kp = Kp - α2·△TP       │         ││
 *      │    │    (减小开度, 减少制冷剂    │         ││
 *      │    │     流量, 提高过热度)       │         ││
 *      │    │  → 输出Kp → 结束           │         ││
 *      │    │                             │         ││
 *      │    ├─ 否 (过热度正常) ──────────┐│         ││
 *      │    │  → -EDT (清除过热度报警)   ││         ││
 *      │    │  → 输出Kp → 结束           ││         ││
 *      │    │    (随过热度变化比例调整)   ││         ││
 *      │    └─────────────────────────────┘│         ││
 *      └───────────────────────────────────┘         ││
 *
 *  参数说明:
 *    △TCZ目标值 = SET_HT_DIFF_TARGET = 6.5℃
 *    △TCZ容差   = SET_HT_DIFF_TOL = 1.5℃ → 范围 5.0 ~ 8.0℃
 *    △TPmin下限 = SET_SH_MIN_LOW = 6.5℃
 *    △TPmin上限 = SET_SH_MIN_HIGH = 8.0℃
 *    α1 = SET_PID_ALPHA1 = 0.5 (传热温差调节系数)
 *    α2 = SET_PID_ALPHA2 = 0.8 (过热度调节系数)
 *
 *  调用时机: 每30秒由主循环调用一次
 * =================================================================== */
void FreqExv_ExvAdjust(void)
{
    /* ---- 读取传感器数据 ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    /* 只在压缩机运行时才调整膨胀阀 */
    if (!(sys_bits & ST_COMP_RUNNING)) {
        return;
    }

    float kp = sensor.VAR_EXV_OPENING;  /* 当前膨胀阀开度 */

    /* ================================================================
     *  第1步: 计算传热温差 △TCZ = 柜温 - 蒸发温度
     *
     *  △TCZ 反映蒸发器与柜内空气之间的热交换效率:
     *    △TCZ大 → 蒸发器过冷, 换热效率低 (结霜等)
     *    △TCZ小 → 蒸发器不够冷, 制冷剂不足
     *    △TCZ在目标范围 → 换热效率正常
     * ================================================================ */
    float delta_tcz = sensor.VAR_CABINET_TEMP - sensor.VAR_EVAP_TEMP;

    /* 同步更新 VAR_HT_DIFF 供其他模块使用 */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_HT_DIFF = delta_tcz;
    SysState_Unlock();

    /* ================================================================
     *  第2步: △TCZ 在 6.5±1.5 范围内 (5.0~8.0℃)?
     *
     *  目标: SET_HT_DIFF_TARGET = 6.5℃
     *  容差: SET_HT_DIFF_TOL = 1.5℃
     *  合格范围: [6.5 - 1.5, 6.5 + 1.5] = [5.0, 8.0]
     * ================================================================ */
    float tcz_low  = SET_HT_DIFF_TARGET - SET_HT_DIFF_TOL;   /* 5.0℃ */
    float tcz_high = SET_HT_DIFF_TARGET + SET_HT_DIFF_TOL;   /* 8.0℃ */

    if (delta_tcz < tcz_low || delta_tcz > tcz_high) {
        /* ============================================================
         *  N → 传热温差不在目标范围
         *
         *  步进调整 (每次 ±EXV_STEP_SIZE = 10步):
         *    △TCZ > 目标 → 关小10步 → 减少制冷剂流量
         *    △TCZ < 目标 → 开大10步 → 增加制冷剂流量
         * ============================================================ */
        if (delta_tcz > SET_HT_DIFF_TARGET) {
            kp = kp - EXV_STEP_SIZE;
        } else {
            kp = kp + EXV_STEP_SIZE;
        }

        EXV_SetOpening(kp);
        return;
    }

    /* ================================================================
     *  Y → 传热温差在目标范围内, 换热效率正常
     *  继续检查过热度安全
     *
     *  第3步: 过热度 △TP < △TPmin?
     *
     *  △TP = VAR_SUPERHEAT (过热度, 由传感器计算)
     *  △TPmin = SET_SH_MIN_LOW = 6.5℃
     *    △TP < 6.5℃ → 过热度过低, 有液击风险
     *
     *  过热度含义:
     *    过热度 = 吸气温度 - 蒸发温度
     *    过热度低 → 制冷剂未完全蒸发, 液态进入压缩机 → 损坏压缩机
     *    过热度高 → 制冷剂完全蒸发, 安全但效率偏低
     * ================================================================ */
    float superheat = sensor.VAR_SUPERHEAT;

    if (superheat < SET_SH_MIN_LOW) {
        /* ============================================================
         *  是 → 过热度过低 (< 6.5℃)
         *
         *  EDT报警 + 关小阀门10步 → 减少制冷剂流量 → 提高过热度
         * ============================================================ */
        g_AlarmFlags |= WARN_SUPERHEAT_LOW;
        kp = kp - EXV_STEP_SIZE;

    } else if (superheat > SET_SH_MIN_HIGH) {
        /* ============================================================
         *  过热度偏高 (> 8.0℃) → 清除报警, 开大阀门10步
         *  增加制冷剂流量, 提高蒸发效率
         * ============================================================ */
        g_AlarmFlags &= ~WARN_SUPERHEAT_LOW;
        kp = kp + EXV_STEP_SIZE;

    } else {
        /* ============================================================
         *  过热度正常 (6.5~8.0℃) → 清除报警, 维持当前开度
         * ============================================================ */
        g_AlarmFlags &= ~WARN_SUPERHEAT_LOW;
    }

    /* ================================================================
     *  第4步: 输出Kp
     * ================================================================ */
    EXV_SetOpening(kp);
}


/* ===================================================================
 *  变频控制和膨胀阀任务主循环
 *
 *  作为FreeRTOS任务运行, 每1秒检查一次PID周期:
 *    PID周期(30s)到时 → 依次执行:
 *      1. PID调整模块 (调节压缩机频率)
 *      2. 膨胀阀调整模块 (调节阀门开度)
 *
 *  循环周期: 1秒 (但实际调整每30秒一次)
 * =================================================================== */
void Task_FreqExv_Process(void const *argument)
{
    (void)argument;

    /* 初始化: 清除PID相关状态 */
    s_prev_delta_t = 0.0f;
    s_prev_dt_valid = false;

    /* 初始化EXV硬件: GPIO配置 + 关阀归零 */
    BSP_EXV_Init();
    BSP_EXV_ResetToZero();

    /* 等待系统初始化完成 */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        /* ============================================
         * 检查PID周期是否到时 (30秒)
         *   TMR_PID_CNT 由定时中断递增,
         *   到达 SET_PID_PERIOD(30s) 时置位 ST_TMR_PID_DONE
         * ============================================ */
        EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

        if (tmr_bits & ST_TMR_PID_DONE) {
            /* ============================================
             * PID周期到时, 执行两个子模块
             * ============================================ */

            /* 子逻辑1: PID调整模块 (调节压缩机频率) */
            FreqExv_PidAdjust();

            /* 子逻辑2: 膨胀阀调整模块 (调节阀门开度) */
            FreqExv_ExvAdjust();

            /* 复位PID周期计时器, 等待下一个30秒 */
            g_TimerData.TMR_PID_CNT = 0;
            xEventGroupClearBits(SysTimerEventGroup, ST_TMR_PID_DONE);
        }

        /* 循环延时: 1秒 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
