#ifndef SYS_STATE_H
#define SYS_STATE_H

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * CO2 制冷系统 — 变量 / 状态标记 / 报警 (来源: 逻辑图 1~6)
 *
 * 命名规则:
 *   VAR_   = 变量测量值 (传感器采样 / 计算中间量)
 *   ST_    = 状态标记   (bool 型, 描述当前系统状态)
 *   TMR_   = 定时计数器 (由定时中断递增, 到时后置位对应 ST_ 标记)
 *   ERR_   = 错误       (严重, 需停机并通知用户)
 *   WARN_  = 警告       (短时可忽略, 持续则升级为错误并通知用户)
 * =========================================================================== */

/* ===================================================================
 *  第一部分: 变量测量值 (VAR_) — 传感器采集 + 计算中间量
 *  来源: 图1/图2/图3/图4/图6
 * =================================================================== */
typedef struct {
    /* --- 图1: 温度控制流程 采集量 --- */
    float VAR_CABINET_TEMP;     /* Tc/T1  冷箱内温度(柜温) (°C)           */
    float VAR_EXHAUST_TEMP;     /* TH     压缩机排气温度 (°C)             */
    float VAR_SUCTION_TEMP;     /* TL     吸气温度 (°C)                   */
    float VAR_COMP_FREQ;        /* F      压缩机当前运行频率 (Hz)          */
    float VAR_SUCTION_PRES;     /* PL     吸气压力 (bar)                  */
    float VAR_DISCHARGE_PRES;   /* PH     排气压力 (bar)                  */
    float VAR_EXV_OPENING;      /* Kp     膨胀阀开度 (步/%)               */
    float VAR_VDC_VOLTAGE;      /* VDC    电源电压 (V)                    */
    float VAR_AMBIENT_TEMP;     /* 环境温度 (°C), 油壳加热/冷凝风机用     */

    /* --- 图2: 除霜流程 + 图3: 蒸发风机 采集量 --- */
    float VAR_EVAP_TEMP;        /* 蒸发器温度 (°C), 除霜判断+风机判断     */

    /* --- 图4: PID/膨胀阀 计算中间量 --- */
    float VAR_DELTA_T;          /* △T = T1(柜温) - Ts(设定温度) (°C)     */
    float VAR_SUPERHEAT;        /* △TP   过热度 (°C)                     */
    float VAR_HT_DIFF;          /* △TCZ  传热温差 = 柜温 - 蒸发温度 (°C) */

    /* --- 图6: 冷凝风机 采集量 --- */
    float VAR_COND_TEMP;        /* 冷凝温度 (°C)                          */

    /* --- 液位传感器 --- */
    uint8_t VAR_LIQUID_LEVEL;   /* 液位状态 (1:有水, 0:没水)              */

    /* --- SHT30 环境温湿度传感器 (I2C1 共用, 地址 0x44) --- */
    float VAR_SHT30_TEMP;       /* SHT30 环境温度 (°C)                    */
    float VAR_SHT30_HUMI;       /* SHT30 相对湿度 (% RH)                  */
} SysVarData_t;


/* ===================================================================
 *  第二部分: 状态标记 (ST_) — 用 FreeRTOS EventGroup 位
 *  来源: 图1/图2/图3/图5/图6
 *
 *  使用方法:
 *    置位: xEventGroupSetBits(SysEventGroup, ST_xxx);
 *    清除: xEventGroupClearBits(SysEventGroup, ST_xxx);
 *    读取: xEventGroupGetBits(SysEventGroup) & ST_xxx
 * =================================================================== */

/* --- 图1: 温度控制 / 压缩机状态 --- */
#define ST_COMP_RUNNING         (1 <<  0)  /* 压缩机正在工作                */
#define ST_SYSTEM_ON            (1 <<  1)  /* 系统开机状态                  */
#define ST_FIRST_RUN            (1 <<  2)  /* 首次运行(通电后第一次启动)    */
#define ST_OIL_HEAT_ON          (1 <<  3)  /* 油壳加热已开启                */
#define ST_WARMUP_DONE          (1 <<  4)  /* 热车时长C20已到时             */

/* --- 图2: 除霜状态 --- */
#define ST_DEFROST_ACTIVE       (1 <<  5)  /* 正在除霜(总标记)              */
#define ST_DEF_HEATING          (1 <<  6)  /* 正在加热(除霜加热阶段)        */
#define ST_DEF_DRIPPING         (1 <<  7)  /* 正在滴水(除霜滴水阶段)        */

/* --- 图3: 蒸发风机状态 --- */
#define ST_EVAP_FAN_ON          (1 <<  8)  /* 蒸发风机正在运行              */
#define ST_EVAP_TEMP_QUAL       (1 <<  9)  /* 蒸发温度合格                  */

/* --- 图4: PID/变频状态 --- */
#define ST_DT_SHRINKING         (1 << 10)  /* △T正在缩小(降频方向)          */

/* --- 图6: 冷凝风机状态 --- */
#define ST_COND_FAN1_ON         (1 << 11)  /* 冷凝风机1正在运行             */
#define ST_COND_FAN2_ON         (1 << 12)  /* 冷凝风机2正在运行             */
#define ST_COND_FAN3_ON         (1 << 13)  /* 冷凝风机3正在运行             */

/* ===================================================================
 *  第三部分: 定时标记 (ST_TMR_) — 由图5定时中断置位
 *  使用第二个 EventGroup, 因为位数可能不够
 * =================================================================== */
#define ST_TMR_TICK_1S          (1 <<  0)  /* 1秒定时到                     */
#define ST_TMR_PID_DONE         (1 <<  1)  /* PID周期(30s)到时              */
#define ST_TMR_C2_DONE          (1 <<  2)  /* 停机保护时长C2到时            */
#define ST_TMR_C3_DONE          (1 <<  3)  /* 通电延迟C3到时                */
#define ST_TMR_C7_DONE          (1 <<  4)  /* 待机保护C7到时                */
#define ST_TMR_C8_DONE          (1 <<  5)  /* 最短运行C8到时                */
#define ST_TMR_DEF_INTV_DONE    (1 <<  6)  /* 除霜间隔到时                  */
#define ST_TMR_DEF_DUR_DONE     (1 <<  7)  /* 除霜加热时长到时              */
#define ST_TMR_DEF_DRIP_DONE    (1 <<  8)  /* 滴水时间到时                  */
#define ST_TMR_WARMUP_DONE      (1 <<  9)  /* 热车时长C20到时               */
#define ST_TMR_EVAP_FAN_DLY     (1 << 10)  /* 蒸发风机F3延时到时            */

/* ===================================================================
 *  第四部分: 定时计数器 (TMR_) — 在定时中断中递增
 * =================================================================== */
typedef struct {
    uint32_t TMR_TICK_1S_CNT;       /* 1秒基准计数                   */
    uint32_t TMR_PID_CNT;           /* PID周期计数                   */
    uint32_t TMR_C2_CNT;            /* 停机时长计数                  */
    uint32_t TMR_C3_CNT;            /* 通电延迟计数                  */
    uint32_t TMR_C7_CNT;            /* 待机保护计数                  */
    uint32_t TMR_C8_CNT;            /* 最短运行计数                  */
    uint32_t TMR_DEF_INTV_CNT;      /* 除霜间隔计数                  */
    uint32_t TMR_DEF_DUR_CNT;       /* 除霜加热时长计数              */
    uint32_t TMR_DRIP_CNT;          /* 滴水时间计数                  */
    uint32_t TMR_WARMUP_CNT;        /* 热车时长计数                  */
    uint32_t TMR_EVAP_FAN_DLY_CNT;  /* 蒸发风机延时计数              */
    uint32_t TMR_LONGRUN_CNT;       /* 连续开机时长计数(警告用)      */
    uint32_t TMR_PRES_HIGH_CNT;     /* 高压超时计数                  */
    uint32_t TMR_PRES_LOW_CNT;      /* 低压超时计数                  */
} SysTimerData_t;

/* ===================================================================
 *  第五部分: 报警标志 (ERR_ / WARN_) — 用位掩码, 存于一个 uint32_t
 *  来源: 图1 告警逻辑 + 图4 PID逻辑
 *
 *  ERR_ = 错误 (严重, 必须停机, 必须通知用户)
 *  WARN_ = 警告 (短时可容忍, 持续时间长则升级通知用户)
 *
 *  使用方法:
 *    置位: g_AlarmFlags |= ERR_xxx;
 *    清除: g_AlarmFlags &= ~ERR_xxx;
 *    检测: if (g_AlarmFlags & ERR_xxx) { ... }
 * =================================================================== */

/* --- 错误 (Error): 立即停机 + 通知用户 --- */
#define ERR_SENSOR_CABINET      (1U <<  0) /* E1   柜温传感器故障           */
#define ERR_VDC_LOW             (1U <<  1) /* EDC  电源电压过低             */
#define ERR_VAC_PHASE           (1U <<  2) /* EAC  VAC错相/断相             */
#define ERR_INV_OVERCURR        (1U <<  3) /* EFI  变频器过流               */
#define ERR_INV_OVERHEAT        (1U <<  4) /* EFT  变频器过热               */
#define ERR_TEMP_LOW_STOP       (1U <<  5) /* ETM  频率过低温度异常停机     */

/* --- 警告 (Warning): 短时容忍, 持续则通知用户 --- */
#define WARN_EXHAUST_HIGH       (1U <<  8) /* WTM  排气温度过高             */
#define WARN_SUCTION_LOW        (1U <<  9) /* WTL  吸气温度过低(连续1-2h)   */
#define WARN_PRES_HIGH          (1U << 10) /* WPH1 排气压力过高             */
#define WARN_PRES_HIGH_TMO      (1U << 11) /* WPH2 高压超时→升级通知       */
#define WARN_PRES_LOW           (1U << 12) /* WPL1 吸气压力过低             */
#define WARN_PRES_LOW_TMO       (1U << 13) /* WPL2 低压超时→升级通知       */
#define WARN_LONGRUN            (1U << 14) /* WLC  连续开机时间过长         */
#define WARN_SUPERHEAT_LOW      (1U << 15) /* EDT  过热度过低               */
#define WARN_NOTIFY_USER        (1U << 16) /* WEN  综合告警-需通知用户      */

/* --- 错误/警告掩码 (方便整体判断) --- */
#define ERR_MASK_ALL            (0x0000003FU)  /* 所有ERR位                 */
#define WARN_MASK_ALL           (0x0001FF00U)  /* 所有WARN位                */

/* ===================================================================
 *  外部全局变量声明
 * =================================================================== */
extern EventGroupHandle_t SysEventGroup;      /* 系统状态标记事件组         */
extern EventGroupHandle_t SysTimerEventGroup;  /* 定时标记事件组            */
extern SemaphoreHandle_t  SensorDataMutex;     /* 传感器数据互斥锁          */

extern volatile uint32_t  g_AlarmFlags;        /* 报警标志字                */
extern SysTimerData_t     g_TimerData;         /* 定时计数器                */

/* ===================================================================
 *  函数接口声明
 * =================================================================== */
void SysState_Init(void);
void SysState_UpdateSensor(SysVarData_t* newData);
void SysState_GetSensor(SysVarData_t* outData);
/* 新增：原子级安全操作接口 */
void SysState_Lock(void);
void SysState_Unlock(void);
SysVarData_t* SysState_GetRawPtr(void);
#endif /* SYS_STATE_H */



