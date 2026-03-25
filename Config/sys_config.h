#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

#include <stdint.h>

/* ===========================================================================
 * CO2 制冷系统 — 全局常量设定值 (来源: 逻辑图 1~6)
 *
 * 命名规则:  SET_<模块>_<含义>
 *   前缀 SET_ 表示"用户可调设定值 / 系统常量"
 * =========================================================================== */

/* ===================================================================
 *  图1  温度控制流程 — 常量设定值
 * =================================================================== */

/* --- 1.1 温度控制与保护 --- */
#define SET_TEMP_TS             (-20.0f)   /* 设定目标柜温 Ts (°C), 范围 -30 ~ 20         */
#define SET_TEMP_HYST_C1        2.0f       /* 温度回差 C1 (°C), 范围 1 ~ 2                 */
#define SET_STOP_TIME_C2        180        /* 停机保护时长 C2 (秒)                          */
#define SET_POWERON_DLY_C3      180        /* 通电延迟时间 C3 (秒), 首次上电防抖            */
#define SET_STBY_TIME_C7        300        /* 待机保护时间 C7 (秒)                          */
#define SET_RUN_MIN_C8          300        /* 最短运行时间 C8 (秒)                          */
#define SET_WARMUP_C20          120        /* 热车时长 C20 (秒), 开机后低速暖机             */

/* --- 1.2 报警阈值 (A 系列) --- */
#define SET_ALARM_A1            (-10.0f)   /* 报警设定值 A1 — 柜温低温报警 (°C)            */
#define SET_ALARM_A2            30.0f      /* 报警设定值 A2 — 柜温高温报警 (°C)            */
#define SET_ALARM_A3            80.0f      /* 报警设定值 A3 — 排温报警 (°C)                */

/* --- 1.3 压缩机保护阈值 --- */
#define SET_EXHAUST_TMAX        110.0f     /* 排气温度上限 Tmax (°C)                       */
#define SET_SUCTION_TMIN        15.0f      /* 吸气温度下限温差 Tmin (°C), 设定值TLS-吸温   */
#define SET_DISCHARGE_PMAX_H    110.0f     /* 排气压力上限-高档 Pmax (bar)                  */
#define SET_DISCHARGE_PMAX_L    70.0f      /* 排气压力上限-低档 Pmax (bar)                  */
#define SET_SUCTION_PMIN        20.0f      /* 吸气压力下限 Pmin (bar)                       */
#define SET_VDC_MIN             300.0f     /* 电源电压下限 VDCmin (V), 待根据硬件确认       */
#define SET_FREQ_INIT           125.0f     /* 开机初始频率 F=125 (Hz)                       */

/* --- 1.4 油壳加热 --- */
#define SET_OIL_HEAT_TEMP       10.0f      /* 油壳加热开启环境温度阈值 (≤10°C 则开)       */

/* --- 1.5 告警超时 --- */
#define SET_WARN_LONGRUN_H      7200       /* 连续开机告警时长-高 (秒), 2小时               */
#define SET_WARN_LONGRUN_L      3600       /* 连续开机告警时长-低 (秒), 1小时               */
#define SET_WARN_PRES_TMO       60         /* 高压/低压超时判定时间 (秒), 待确认            */

/* ===================================================================
 *  图2  除霜流程 — 常量设定值
 * =================================================================== */
#define SET_DEF_INTERVAL        (3 * 3600) /* 除霜间隔时间 (秒), 2~3小时                   */
#define SET_DEF_MIN_INTV        (2 * 3600) /* 除霜最短间隔时间 (秒), 2小时                 */
#define SET_DEF_HEAT_MAX        (45 * 60)  /* 加热最大时长 (秒), 45分钟                    */
#define SET_DEF_HEAT_TLIMIT     10.0f      /* 加热温度上限/除霜退出温度 (°C)               */
#define SET_DRIP_TIME           (5 * 60)   /* 滴水时间 (秒), 5分钟                         */
#define SET_DEF_COMP_DLY        15         /* 除霜后压缩机启动延迟 (秒)                    */
#define SET_DEF_STEP_DLY        30         /* 加热子程序延时步进X (秒)                     */

/* ===================================================================
 *  图3  蒸发风机(1控6)流程 — 常量设定值
 * =================================================================== */
#define SET_EVAP_FAN_MODE       1          /* 蒸发风机运行模式 F1 (1/2/4)                  */
#define SET_EVAP_FAN_DLY        30         /* 蒸发风机延时启动 F3 (秒)                     */
#define SET_EVAP_TEMP_OK        (-15.0f)   /* 蒸发温度合格阈值 (°C)                       */

/* ===================================================================
 *  图4  变频控制(PID)和膨胀阀流程 — 常量设定值
 * =================================================================== */
#define SET_DT_MAX              5.0f       /* △Tmax: 最快升频温差阈值 (°C)                */
#define SET_DT_MIN              1.0f       /* △Tmin: 比例调节温差下限 (°C)                */
#define SET_FREQ_MIN            20.0f      /* Fmin: 压缩机最低运行频率 (Hz)                */
#define SET_SH_MIN_LOW          6.5f       /* 过热度下限-低值 △TPmin (°C)                 */
#define SET_SH_MIN_HIGH         8.0f       /* 过热度下限-高值 △TPmin (°C)                 */
#define SET_HT_DIFF_TARGET      6.5f       /* 传热温差 △TCZ 目标值 (°C)                   */
#define SET_HT_DIFF_TOL         1.5f       /* 传热温差 △TCZ 容差 (±1.5°C)                */
#define SET_PID_ALPHA1          0.5f       /* α1: 传热温差调整系数                         */
#define SET_PID_ALPHA2          0.8f       /* α2: 过热度调整系数                            */
#define SET_PID_PERIOD          30         /* PID计算周期 (秒)                              */

/* ===================================================================
 *  图5  定时中断服务 — 无额外常量 (引用上述 C2/C3/C7/C8/除霜间隔等)
 * =================================================================== */

/* ===================================================================
 *  图6  冷凝风机(3台)流程 — 常量设定值
 * =================================================================== */
#define SET_COND_FAN_ON_T       35.0f      /* 冷凝风机开启温度 (°C)                        */
#define SET_COND_FAN_OFF_T      25.0f      /* 冷凝风机关闭温度 (°C)                        */

/* ===================================================================
 *  存储日志结构体 & EEPROM 映射 (保持原有)
 * =================================================================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t Year;
    uint8_t Month;
    uint8_t Date;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;

    float   EvapTemp;
    float   SetTemp;
    float   CondTemp;

    uint8_t EventType;
    uint8_t CheckSum;
} SysLog_t;
#pragma pack(pop)

#define LOG_MAX_COUNT       100
#define LOG_SIZE            20
#define EEPROM_INDEX_ADDR   0x0000
#define EEPROM_DATA_START   0x0010

#endif /* SYS_CONFIG_H */



