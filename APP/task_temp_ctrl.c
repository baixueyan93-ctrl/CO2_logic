#include "task_temp_ctrl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "sys_state.h"
#include "sys_config.h"
#include <stdbool.h>

/* ===========================================================================
 * 温度控制流程 (逻辑图1) — 实现文件
 *
 * 子逻辑:
 *   1. TempCtrl_ShutdownAlarm()    停机异常逻辑告警
 *   2. TempCtrl_CompressorStart()  压缩机开机逻辑
 *   3. TempCtrl_OilHeatControl()   油壳加热逻辑
 *   4. TempCtrl_AlarmProcess()     告警(温度压力)处理逻辑
 * =========================================================================== */

/* ===================================================================
 *  内部辅助函数 / 硬件操作桩 (待硬件资料确认后实现)
 * =================================================================== */

/* --- 油壳加热继电器 K2 控制 --- */
static void OilHeater_On(void)
{
    HAL_GPIO_WritePin(OIL_HEATER_GPIO_PORT, OIL_HEATER_GPIO_PIN, GPIO_PIN_SET);
    xEventGroupSetBits(SysEventGroup, ST_OIL_HEAT_ON);
}

static void OilHeater_Off(void)
{
    HAL_GPIO_WritePin(OIL_HEATER_GPIO_PORT, OIL_HEATER_GPIO_PIN, GPIO_PIN_RESET);
    xEventGroupClearBits(SysEventGroup, ST_OIL_HEAT_ON);
}

/* --- 压缩机启动 (通过变频器, 接口待确认) --- */
static void Compressor_Start(void)
{
    /* TODO: 发送变频器启动指令, 设置初始频率 F=125Hz
     * 可能通过 RS485/MODBUS 或 GPIO 控制
     */
    xEventGroupSetBits(SysEventGroup, ST_COMP_RUNNING);
}

/* --- 压缩机停止 --- */
static void Compressor_Stop(void)
{
    /* TODO: 发送变频器停止指令 */
    xEventGroupClearBits(SysEventGroup, ST_COMP_RUNNING);
}

/* --- 设置压缩机频率 --- */
static void Compressor_SetFreq(float freq_hz)
{
    /* TODO: 通过变频器通信设置频率
     * freq_hz: 目标频率 (Hz)
     */
    SysState_Lock();
    SysState_GetRawPtr()->VAR_COMP_FREQ = freq_hz;
    SysState_Unlock();
}

/* --- 读取VAC相序状态 (接口待确认) ---
 * 返回: true = 相序正常, false = 错相/断相
 */
static bool VAC_PhaseOK(void)
{
    /* TODO: 读取VAC错/断相检测GPIO或通信接口
     * 可能来自变频器反馈信号
     */
    return true;  /* 占位: 默认正常 */
}

/* --- 读取变频器过流状态 ---
 * 返回: true = 过流故障, false = 正常
 */
static bool Inverter_IsOvercurrent(void)
{
    /* TODO: 从变频器读取过流状态
     * 可能通过GPIO输入或MODBUS通信
     */
    return false;  /* 占位: 默认正常 */
}

/* --- 读取变频器过热状态 ---
 * 返回: true = 过热故障, false = 正常
 */
static bool Inverter_IsOverheat(void)
{
    /* TODO: 从变频器读取过热状态 */
    return false;  /* 占位: 默认正常 */
}

/* --- 通知用户 (蜂鸣器 / RS485 / 面板显示) --- */
static void NotifyUser(uint32_t alarm_code)
{
    /* TODO: 根据告警码通知用户
     * 可通过面板显示故障码 / 蜂鸣器报警 / RS485上报
     */
    (void)alarm_code;
}


/* ===================================================================
 *  子逻辑1: 停机异常逻辑告警
 *
 *  流程图 (1.1):
 *    开始
 *    → 采集电源电压VDC (PB1/VSININ)
 *    → 读取电源电压下限VDCmin
 *    → VDC ≥ VDCmin?
 *        Y → 清除 -EDC
 *        N → 置位 EDC, 关机状态
 *    → VAC错/断相?
 *        Y → 清除 -EAC
 *        N → 置位 EAC, 关机状态
 *    → 变频器过流?
 *        Y → 清除 -EFI
 *        N → 置位 EFI, 关机状态
 *    → 变频器过热?
 *        Y → 清除 -EFT
 *        N → 置位 EFT, 关机状态
 *    → 通知用户
 *    → 结束
 * =================================================================== */
void TempCtrl_ShutdownAlarm(void)
{
    SysVarData_t sensor;
    bool need_shutdown = false;

    /* ---- 第1步: 采集电源电压 VDC (来自 PB1/VSININ, ADC采集) ---- */
    SysState_GetSensor(&sensor);
    float vdc = sensor.VAR_VDC_VOLTAGE;

    /* ---- 第2步: 判断 VDC ≥ VDCmin ---- */
    if (vdc >= SET_VDC_MIN) {
        /* 电压正常 → 清除EDC告警 */
        g_AlarmFlags &= ~ERR_VDC_LOW;
    } else {
        /* 电压偏低 → 置位EDC, 需要关机 */
        g_AlarmFlags |= ERR_VDC_LOW;
        need_shutdown = true;
    }

    /* ---- 第3步: 判断 VAC 错/断相 ---- */
    if (VAC_PhaseOK()) {
        /* 相序正常 → 清除EAC告警 */
        g_AlarmFlags &= ~ERR_VAC_PHASE;
    } else {
        /* 错相/断相 → 置位EAC, 需要关机 */
        g_AlarmFlags |= ERR_VAC_PHASE;
        need_shutdown = true;
    }

    /* ---- 第4步: 判断 变频器过流 ---- */
    if (!Inverter_IsOvercurrent()) {
        /* 正常 → 清除EFI告警 */
        g_AlarmFlags &= ~ERR_INV_OVERCURR;
    } else {
        /* 过流 → 置位EFI, 需要关机 */
        g_AlarmFlags |= ERR_INV_OVERCURR;
        need_shutdown = true;
    }

    /* ---- 第5步: 判断 变频器过热 ---- */
    if (!Inverter_IsOverheat()) {
        /* 正常 → 清除EFT告警 */
        g_AlarmFlags &= ~ERR_INV_OVERHEAT;
    } else {
        /* 过热 → 置位EFT, 需要关机 */
        g_AlarmFlags |= ERR_INV_OVERHEAT;
        need_shutdown = true;
    }

    /* ---- 第6步: 如有异常 → 停机 + 通知用户 ---- */
    if (need_shutdown) {
        Compressor_Stop();
        xEventGroupClearBits(SysEventGroup, ST_SYSTEM_ON);
        NotifyUser(g_AlarmFlags & ERR_MASK_ALL);
    }
}


/* ===================================================================
 *  子逻辑2: 压缩机开机逻辑
 *
 *  流程图 (1.1):
 *    开启压缩机
 *    → F = 125 (初始频率125Hz)
 *    → 热车时长C20到时?
 *        N → 继续等待 (保持F=125运行)
 *        Y → 进入PID调整模块
 *    → PID计算, 幅度限制及输出
 *    → 结束
 *
 *  说明: 此函数在确认无异常后被调用, 启动压缩机并等待热车完成
 * =================================================================== */
void TempCtrl_CompressorStart(void)
{
    EventBits_t sys_bits  = xEventGroupGetBits(SysEventGroup);
    EventBits_t tmr_bits  = xEventGroupGetBits(SysTimerEventGroup);

    /* 如果压缩机已经在运行, 不重复启动 */
    if (sys_bits & ST_COMP_RUNNING) {
        return;
    }

    /* ---- 第1步: 开启压缩机, 设定初始频率 F=125Hz ---- */
    Compressor_Start();
    Compressor_SetFreq(SET_FREQ_INIT);  /* F = 125Hz */

    /* ---- 第2步: 启动热车计时器 C20 ----
     * 热车计时由定时中断服务(逻辑图5)中的1s定时器驱动:
     *   g_TimerData.TMR_WARMUP_CNT 每秒递增
     *   到达 SET_WARMUP_C20 时置位 ST_TMR_WARMUP_DONE
     *
     * 这里复位计数器, 开始计时
     */
    g_TimerData.TMR_WARMUP_CNT = 0;
    xEventGroupClearBits(SysTimerEventGroup, ST_TMR_WARMUP_DONE);
    xEventGroupClearBits(SysEventGroup, ST_WARMUP_DONE);

    /* ---- 第3步: 等待热车完成 (在主循环中轮询) ----
     * 注意: 不在此处阻塞等待, 而是由主循环周期性检查
     *       热车期间保持 F=125Hz 运转
     *       热车完成后主循环将调用PID调整模块
     */
}

/* --- 热车状态检查 (供主循环调用) ---
 * 返回: true = 热车完成, 可以进入PID; false = 仍在热车中
 */
static bool CompressorWarmup_IsDone(void)
{
    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);

    if (tmr_bits & ST_TMR_WARMUP_DONE) {
        /* 热车完成, 置位系统状态标志 */
        xEventGroupSetBits(SysEventGroup, ST_WARMUP_DONE);
        return true;
    }
    return false;
}

/* --- PID调整模块 (框架桩) ---
 * 热车完成后, 由此函数根据温度偏差计算压缩机频率
 */
static void TempCtrl_PID_Adjust(void)
{
    /* TODO: 完整PID实现将在 逻辑图4(变频控制和膨胀阀流程) 中展开
     *
     * 基本框架:
     *   1. 计算偏差: deltaT = VAR_CABINET_TEMP - SET_TEMP_TS
     *   2. PID计算:  output = Kp*e + Ki*∫e + Kd*de/dt
     *   3. 幅度限制: clamp(output, SET_FREQ_MIN, 最大频率)
     *   4. 输出频率: Compressor_SetFreq(output)
     */

    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    float delta_t = sensor.VAR_CABINET_TEMP - SET_TEMP_TS;

    /* 简化框架: 后续由逻辑图4完善 */
    float target_freq = SET_FREQ_INIT;  /* 占位 */

    /* 频率下限保护 */
    if (target_freq < SET_FREQ_MIN) {
        target_freq = SET_FREQ_MIN;
    }

    Compressor_SetFreq(target_freq);
}


/* ===================================================================
 *  子逻辑4: 告警（温度压力）处理逻辑
 *
 *  流程图 (1.温度控制流程.pdf 从左数第二个):
 *
 *    开始
 *    → 1s到时?  N→等待  Y→继续
 *    → 采集: T1(冷箱内温度), TH(排气温度), TL(吸气温度),
 *            F(压缩机频率), PL(吸气压力), PH(排气压力), Kp(膨胀阀开度)
 *    → 读取: TS(设定温度), Tmax(排温上限), Tmin(吸温下限), VDCmin
 *
 *    ① 排温 TH ≥ Tmax [110℃]?
 *        Y → WTM (排温过高告警)
 *        N → -(WTM)
 *
 *    ② 连续开机时长 ≥ 1-2小时?
 *        Y → WLC (长时间运行告警)
 *        N → -(WLC)
 *
 *    ③ 设定值TLS - 吸温TL ≤ Tmin [5-10℃, 温差15]?
 *        Y → WTL (吸温过低告警)
 *        N → -(WTL)
 *
 *    ④ 排压 PH ≥ Pmax [110/70]?
 *        Y → WPH1 → 高压超时? → Y:WPH2  N:-(WPH2)
 *        N → -(WPH1), -(WPH2), 复位高压计时
 *
 *    ⑤ 吸压 PL ≤ Pmin [20]?
 *        Y → WPL1 → 低压超时? → Y:WPL2  N:-(WPL2)
 *        N → -(WPL1), -(WPL2), 复位低压计时
 *
 *    ⑥ 汇总: WPL|WPH|WTM|WTL|WLC 任一存在?
 *        Y → WEN (综合告警, 通知用户)
 *        N → -(WEN)
 *    → 结束
 *
 *  调用时机: 每1秒由主循环调用一次 (与流程图"1s到时?"对应)
 *  注意: 本逻辑只产生WARN级别告警, 不直接停机;
 *        停机由子逻辑1(停机异常逻辑告警)的ERR级别负责
 * =================================================================== */
void TempCtrl_AlarmProcess(void)
{
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    /* ================================================================
     * ① 排温 TH ≥ Tmax [110℃]?
     *    排气温度过高说明压缩比过大或制冷剂不足
     * ================================================================ */
    if (sensor.VAR_EXHAUST_TEMP >= SET_EXHAUST_TMAX) {
        /* Y → WTM 置位 */
        g_AlarmFlags |= WARN_EXHAUST_HIGH;
    } else {
        /* N → -(WTM) 清除 */
        g_AlarmFlags &= ~WARN_EXHAUST_HIGH;
    }

    /* ================================================================
     * ② 连续开机时长 ≥ 1-2小时?
     *    TMR_LONGRUN_CNT 由定时中断服务(逻辑图5)在压缩机运行期间每秒递增
     *    压缩机停止时由其他逻辑复位
     *
     *    流程图标注"1-2小时":
     *      SET_WARN_LONGRUN_L = 3600s (1小时, 初级告警)
     *      SET_WARN_LONGRUN_H = 7200s (2小时, 严重告警)
     *    此处用低阈值(1小时)触发告警, 符合流程图"≥1-2小时"
     * ================================================================ */
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    if (sys_bits & ST_COMP_RUNNING) {
        /* 压缩机运行中, 检查累计运行时长 */
        if (g_TimerData.TMR_LONGRUN_CNT >= SET_WARN_LONGRUN_L) {
            /* Y → WLC 置位 */
            g_AlarmFlags |= WARN_LONGRUN;
        } else {
            /* N → -(WLC) 清除 */
            g_AlarmFlags &= ~WARN_LONGRUN;
        }
    } else {
        /* 压缩机未运行, 清除长运行告警并复位计时 */
        g_AlarmFlags &= ~WARN_LONGRUN;
        g_TimerData.TMR_LONGRUN_CNT = 0;
    }

    /* ================================================================
     * ③ 设定值TLS - 吸温TL ≤ Tmin [5-10℃, 温差15]?
     *    含义: 吸气温度过低 → 吸气过热度不足, 有液击风险
     *    计算: (设定蒸发温度TLS) - (实际吸气温度TL)
     *    如果差值 ≤ Tmin(15℃), 说明吸温偏低
     *
     *    SET_SUCTION_TMIN = 15.0f (温差阈值)
     *    TLS 近似取 SET_TEMP_TS (设定温度)
     * ================================================================ */
    float suction_diff = SET_TEMP_TS - sensor.VAR_SUCTION_TEMP;

    if (suction_diff <= SET_SUCTION_TMIN) {
        /* Y → WTL 置位: 吸温与设定值差距过小, 吸气过热度不足 */
        g_AlarmFlags |= WARN_SUCTION_LOW;
    } else {
        /* N → -(WTL) 清除 */
        g_AlarmFlags &= ~WARN_SUCTION_LOW;
    }

    /* ================================================================
     * ④ 排压 PH ≥ Pmax [110/70]?
     *    流程图标注两个值: 高档110bar, 低档70bar
     *    此处用 SET_DISCHARGE_PMAX_L (70bar) 作为告警触发阈值
     *
     *    分两级:
     *      WPH1 = 排压超限 (即时告警)
     *      WPH2 = 排压超限且持续超时 (严重告警)
     * ================================================================ */
    if (sensor.VAR_DISCHARGE_PRES >= SET_DISCHARGE_PMAX_L) {
        /* Y → WPH1 置位: 排压超限 */
        g_AlarmFlags |= WARN_PRES_HIGH;

        /* 高压超时? — TMR_PRES_HIGH_CNT 由定时中断每秒递增 */
        if (g_TimerData.TMR_PRES_HIGH_CNT >= SET_WARN_PRES_TMO) {
            /* Y → WPH2 置位: 高压持续超时 */
            g_AlarmFlags |= WARN_PRES_HIGH_TMO;
        } else {
            /* N → -(WPH2) */
            g_AlarmFlags &= ~WARN_PRES_HIGH_TMO;
        }
    } else {
        /* N → -(WPH1), -(WPH2), 复位高压计时 */
        g_AlarmFlags &= ~WARN_PRES_HIGH;
        g_AlarmFlags &= ~WARN_PRES_HIGH_TMO;
        g_TimerData.TMR_PRES_HIGH_CNT = 0;
    }

    /* ================================================================
     * ⑤ 吸压 PL ≤ Pmin [20]?
     *    吸气压力过低说明制冷剂不足或膨胀阀开度不够
     *
     *    分两级:
     *      WPL1 = 吸压过低 (即时告警)
     *      WPL2 = 吸压过低且持续超时 (严重告警)
     * ================================================================ */
    if (sensor.VAR_SUCTION_PRES <= SET_SUCTION_PMIN) {
        /* Y → WPL1 置位: 吸压过低 */
        g_AlarmFlags |= WARN_PRES_LOW;

        /* 低压超时? — TMR_PRES_LOW_CNT 由定时中断每秒递增 */
        if (g_TimerData.TMR_PRES_LOW_CNT >= SET_WARN_PRES_TMO) {
            /* Y → WPL2 置位: 低压持续超时 */
            g_AlarmFlags |= WARN_PRES_LOW_TMO;
        } else {
            /* N → -(WPL2) */
            g_AlarmFlags &= ~WARN_PRES_LOW_TMO;
        }
    } else {
        /* N → -(WPL1), -(WPL2), 复位低压计时 */
        g_AlarmFlags &= ~WARN_PRES_LOW;
        g_AlarmFlags &= ~WARN_PRES_LOW_TMO;
        g_TimerData.TMR_PRES_LOW_CNT = 0;
    }

    /* ================================================================
     * ⑥ 汇总: WPL|WPH|WTM|WTL|WLC 任一存在 → WEN通知用户
     *    流程图最底部: 所有告警汇聚到一个判断
     * ================================================================ */
    if (g_AlarmFlags & WARN_MASK_ALL) {
        /* 存在任意WARN → WEN 置位, 通知用户 */
        g_AlarmFlags |= WARN_NOTIFY_USER;
        NotifyUser(g_AlarmFlags & WARN_MASK_ALL);
    } else {
        /* 全部正常 → -(WEN) 清除 */
        g_AlarmFlags &= ~WARN_NOTIFY_USER;
    }
}


/* ===================================================================
 *  子逻辑3: 油壳加热逻辑
 *
 *  流程图 (1.1):
 *    开始
 *    → 压缩机开机?
 *        Y → 油壳加热关 → 结束
 *        N → 环境温度 ≤ 10°C?
 *            Y → 油壳加热开 → 结束
 *            N → 油壳加热关 → 结束
 *
 *  硬件: K2 继电器 → 滑油热丝 (PSD0D)
 * =================================================================== */
void TempCtrl_OilHeatControl(void)
{
    EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

    /* ---- 判断: 压缩机是否在运行 ---- */
    if (sys_bits & ST_COMP_RUNNING) {
        /* 压缩机运行中 → 油壳加热关闭 (压缩机自身产热, 无需额外加热) */
        OilHeater_Off();
        return;
    }

    /* ---- 压缩机未运行 → 判断环境温度 ---- */
    SysVarData_t sensor;
    SysState_GetSensor(&sensor);

    if (sensor.VAR_AMBIENT_TEMP <= SET_OIL_HEAT_TEMP) {
        /* 环境温度 ≤ 10°C → 油壳加热开 (防止冷凝液稀释润滑油) */
        OilHeater_On();
    } else {
        /* 环境温度 > 10°C → 油壳加热关 */
        OilHeater_Off();
    }
}


/* ===================================================================
 *  温度控制任务主循环
 *
 *  作为FreeRTOS任务运行, 周期性执行子逻辑:
 *    1. 停机异常检测 (每次循环, 最高优先级)
 *    2. 告警(温度压力)处理 (每次循环)
 *    3. 压缩机开机/运行管理
 *    4. 油壳加热管理
 *
 *  循环周期: 1秒 (与逻辑图中1s采集周期对应)
 * =================================================================== */
void Task_TempCtrl_Process(void const *argument)
{
    (void)argument;

    /* 等待系统初始化完成, 传感器数据稳定 */
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        /* ============================================
         * 第1步: 停机异常逻辑告警 (最高优先级, 每次必检)
         * ============================================ */
        TempCtrl_ShutdownAlarm();

        /* ============================================
         * 第2步: 告警(温度压力)处理逻辑 (每次循环)
         *   WARN级别: 不直接停机, 但通知用户
         * ============================================ */
        TempCtrl_AlarmProcess();

        /* 如果有严重故障(ERR级别), 跳过压缩机管理 */
        if (g_AlarmFlags & ERR_MASK_ALL) {
            /* 确保油壳加热也被正确管理 (即使故障状态) */
            TempCtrl_OilHeatControl();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* ============================================
         * 第3步: 压缩机开机/运行逻辑
         * ============================================ */
        EventBits_t sys_bits = xEventGroupGetBits(SysEventGroup);

        if (sys_bits & ST_SYSTEM_ON) {
            if (!(sys_bits & ST_COMP_RUNNING)) {
                /* 系统开启但压缩机未运行 → 启动压缩机 */
                TempCtrl_CompressorStart();
            } else {
                /* 压缩机运行中 → 检查热车/PID */
                if (CompressorWarmup_IsDone()) {
                    /* 热车完成 → PID调整 */
                    EventBits_t tmr_bits = xEventGroupGetBits(SysTimerEventGroup);
                    if (tmr_bits & ST_TMR_PID_DONE) {
                        TempCtrl_PID_Adjust();
                        /* 复位PID周期计时器 */
                        g_TimerData.TMR_PID_CNT = 0;
                        xEventGroupClearBits(SysTimerEventGroup, ST_TMR_PID_DONE);
                    }
                }
                /* else: 热车中, 保持 F=125Hz */
            }
        }

        /* ============================================
         * 第4步: 油壳加热逻辑
         * ============================================ */
        TempCtrl_OilHeatControl();

        /* ============================================
         * 循环延时: 1秒
         * ============================================ */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
