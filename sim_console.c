/*
 * sim_console.c - CO2制冷系统 独立命令行模拟器
 *
 * 功能: 不需要搭建嵌入式系统, 在普通电脑上编译运行,
 *       输入传感器模拟值, 即可看到压缩机频率和电子膨胀阀开度的调节结果。
 *
 * 编译: gcc sim_console.c -o sim_console -lm
 * 运行: ./sim_console
 *
 * 控制逻辑来源: 逻辑图4 (变频调节 + 阀门调节)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ===================== 系统参数 (来自 sys_config.h) ===================== */
#define SET_TEMP_TS           (-10.0f)  /* 默认设定温度 (C)         */
#define SET_FREQ_MIN          120.0f    /* 压缩机最低频率 (Hz)      */
#define SET_FREQ_MAX          320.0f    /* 压缩机最高频率 (Hz)      */
#define SET_FREQ_INIT         120.0f    /* 压缩机启动频率 (Hz)      */
#define SET_EXV_MAX           500.0f    /* 膨胀阀最大开度 (步)      */
#define SET_EXV_MIN           0.0f     /* 膨胀阀最小开度 (步)      */
#define SET_EXV_INIT          250.0f    /* 膨胀阀初始开度 (步)      */
#define SET_HT_DIFF_TARGET    6.5f     /* 传热温差目标值 (C)        */
#define SET_HT_DIFF_LO        5.0f     /* 传热温差下限 (C)          */
#define SET_HT_DIFF_HI        8.0f     /* 传热温差上限 (C)          */
#define SET_PID_ALPHA1        0.5f     /* 传热温差调节系数          */
#define SET_PID_ALPHA2        0.8f     /* 过热度调节系数            */
#define SET_SH_MIN            5.0f     /* 过热度最低安全值 (C)      */
#define SET_EXHAUST_TMAX      110.0f   /* 排气温度报警上限 (C)      */
#define SET_DISCHARGE_PMAX    70.0f    /* 排气压力报警上限 (bar)    */
#define SET_SUCTION_PMIN      20.0f    /* 吸气压力报警下限 (bar)    */

/* ===================== 模拟状态 ===================== */
typedef struct {
    /* 传感器输入 */
    float cabinet_temp;     /* 柜温 (C)               */
    float set_temp;         /* 设定温度 (C)            */
    float evap_temp;        /* 蒸发器温度 (C)          */
    float suction_temp;     /* 吸气温度 (C)            */
    float discharge_temp;   /* 排气温度 (C)            */
    float sat_temp_low;     /* 低压饱和温度 (C)        */
    float discharge_pres;   /* 排气压力 (bar)          */
    float suction_pres;     /* 吸气压力 (bar)          */

    /* 控制输出 */
    float comp_freq;        /* 压缩机当前频率 (Hz)     */
    float exv_opening;      /* 膨胀阀当前开度 (步)     */

    /* 计算中间值 */
    float delta_t;          /* 温差 = 柜温 - 设定温度  */
    float superheat;        /* 过热度 = 吸气温度 - 低压饱和温度 */
    float delta_tcz;        /* 传热温差 = 柜温 - 蒸发器温度 */

    /* 报警标志 */
    int alarm_exhaust_high; /* 排气温度过高             */
    int alarm_pres_high;    /* 排气压力过高             */
    int alarm_pres_low;     /* 吸气压力过低             */
    int alarm_superheat_low;/* 过热度过低 (液击风险)    */

    /* 调节细节 */
    int   freq_step;        /* 频率调节步长 (Hz/s)      */
    int   exv_action;       /* 阀门动作: 0=不动 1=温差调 2=过热度调 */
    float tcz_err;          /* 传热温差偏差             */
} SimState_t;

/* ===================== 辅助函数 ===================== */
static float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ===================== 频率调节逻辑 (每1秒执行一次) ===================== */
static void sim_freq_adjust(SimState_t *s)
{
    s->delta_t = s->cabinet_temp - s->set_temp;

    s->freq_step = 0;
    if      (s->delta_t >= 5.0f)  s->freq_step =  3;  /* 温差>=5C: +3Hz/s  */
    else if (s->delta_t >= 2.0f)  s->freq_step =  2;  /* 温差2~5C: +2Hz/s  */
    else if (s->delta_t >  0.0f)  s->freq_step =  1;  /* 温差0~2C: +1Hz/s  */
    else if (s->delta_t <= -5.0f) s->freq_step = -3;  /* 温差<=-5C: -3Hz/s */
    else if (s->delta_t <= -2.0f) s->freq_step = -2;  /* 温差-5~-2C: -2Hz/s*/
    else if (s->delta_t <  0.0f)  s->freq_step = -1;  /* 温差-2~0C: -1Hz/s */

    s->comp_freq = clampf(s->comp_freq + s->freq_step,
                          SET_FREQ_MIN, SET_FREQ_MAX);
}

/* ===================== 膨胀阀调节逻辑 (每30秒执行一次) ===================== */
static void sim_exv_adjust(SimState_t *s)
{
    s->superheat = s->suction_temp - s->sat_temp_low;
    s->delta_tcz = s->cabinet_temp - s->evap_temp;
    s->tcz_err   = 0.0f;
    s->exv_action = 0;
    s->alarm_superheat_low = 0;

    if (s->delta_tcz < SET_HT_DIFF_LO || s->delta_tcz > SET_HT_DIFF_HI) {
        /* 传热温差超出 [5.0, 8.0] 范围 -> 调整阀门 */
        s->tcz_err = s->delta_tcz - SET_HT_DIFF_TARGET;
        s->exv_opening = s->exv_opening - SET_PID_ALPHA1 * s->tcz_err;
        s->exv_action = 1;
    } else {
        /* 传热温差OK, 检查过热度 */
        if (s->superheat < SET_SH_MIN) {
            /* 过热度过低 -> EDT报警, 减小开度防止液击 */
            s->exv_opening = s->exv_opening - SET_PID_ALPHA2 * s->superheat;
            s->exv_action = 2;
            s->alarm_superheat_low = 1;
        } else {
            /* 全部正常, 维持当前开度 */
            s->exv_action = 0;
        }
    }

    s->exv_opening = clampf(s->exv_opening, SET_EXV_MIN, SET_EXV_MAX);
}

/* ===================== 报警检测 ===================== */
static void sim_alarm_check(SimState_t *s)
{
    s->alarm_exhaust_high = (s->discharge_temp >= SET_EXHAUST_TMAX) ? 1 : 0;
    s->alarm_pres_high    = (s->discharge_pres >= SET_DISCHARGE_PMAX) ? 1 : 0;
    s->alarm_pres_low     = (s->suction_pres <= SET_SUCTION_PMIN) ? 1 : 0;
}

/* ===================== 打印分割线 ===================== */
static void print_separator(void)
{
    printf("================================================================\n");
}

/* ===================== 打印当前状态 ===================== */
static void print_state(const SimState_t *s)
{
    print_separator();
    printf("              [ 控制输出结果 ]\n");
    print_separator();

    /* 压缩机 */
    printf("\n  --- 压缩机 ---\n");
    printf("  温差 dT          = %+.1f C  (柜温%.1f - 设定%.1f)\n",
           s->delta_t, s->cabinet_temp, s->set_temp);
    printf("  频率调节步长     = %+d Hz/s\n", s->freq_step);
    printf("  >>> 压缩机频率   = %.1f Hz  [范围 %.0f~%.0f]\n",
           s->comp_freq, SET_FREQ_MIN, SET_FREQ_MAX);

    /* 膨胀阀 */
    printf("\n  --- 电子膨胀阀 (EXV) ---\n");
    printf("  传热温差 dTcz    = %.1f C  (柜温%.1f - 蒸发%.1f)\n",
           s->delta_tcz, s->cabinet_temp, s->evap_temp);
    printf("  传热温差偏差     = %+.2f C  (目标 %.1f, 范围 [%.1f, %.1f])\n",
           s->tcz_err, SET_HT_DIFF_TARGET, SET_HT_DIFF_LO, SET_HT_DIFF_HI);
    printf("  过热度           = %.1f C  (吸气%.1f - 饱和%.1f)\n",
           s->superheat, s->suction_temp, s->sat_temp_low);

    const char *action_str[] = { "不调整(正常)", "传热温差调整", "过热度报警调整" };
    printf("  调节动作         = %s\n",
           (s->exv_action >= 0 && s->exv_action <= 2) ?
           action_str[s->exv_action] : "未知");
    printf("  >>> 膨胀阀开度   = %.1f / 500 步  (%.1f%%)\n",
           s->exv_opening, s->exv_opening / 5.0f);

    /* 报警 */
    printf("\n  --- 报警状态 ---\n");
    printf("  排气温度过高 (WTM)  : %s  (当前%.1fC, 阈值%.0fC)\n",
           s->alarm_exhaust_high ? "[!!!报警!!!]" : "[正常]",
           s->discharge_temp, SET_EXHAUST_TMAX);
    printf("  排气压力过高 (WPH)  : %s  (当前%.1fbar, 阈值%.0fbar)\n",
           s->alarm_pres_high ? "[!!!报警!!!]" : "[正常]",
           s->discharge_pres, SET_DISCHARGE_PMAX);
    printf("  吸气压力过低 (WPL)  : %s  (当前%.1fbar, 阈值%.0fbar)\n",
           s->alarm_pres_low ? "[!!!报警!!!]" : "[正常]",
           s->suction_pres, SET_SUCTION_PMIN);
    printf("  过热度过低   (EDT)  : %s  (当前%.1fC, 最低%.1fC)\n",
           s->alarm_superheat_low ? "[!!!报警!!!]" : "[正常]",
           s->superheat, SET_SH_MIN);
    print_separator();
}

/* ===================== 读取一个浮点输入 ===================== */
static float read_float(const char *prompt, float current)
{
    char buf[64];
    printf("  %s [当前=%.1f, 直接回车保持]: ", prompt, current);
    fflush(stdout);
    if (fgets(buf, sizeof(buf), stdin) == NULL) return current;
    if (buf[0] == '\n' || buf[0] == '\r') return current;
    return (float)atof(buf);
}

/* ===================== 主函数 ===================== */
int main(void)
{
    SimState_t s;
    char buf[64];
    int sim_seconds = 0;

    /* 默认初始值 */
    memset(&s, 0, sizeof(s));
    s.cabinet_temp   =  10.0f;
    s.set_temp       = SET_TEMP_TS;    /* -10 C */
    s.evap_temp      =  -2.0f;
    s.suction_temp   =   2.0f;
    s.discharge_temp =  80.0f;
    s.sat_temp_low   =  -8.0f;
    s.discharge_pres =  50.0f;
    s.suction_pres   =  30.0f;
    s.comp_freq      = SET_FREQ_INIT;  /* 120 Hz */
    s.exv_opening    = SET_EXV_INIT;   /* 250 步 */

    printf("\n");
    print_separator();
    printf("    CO2 跨临界制冷系统 - 命令行模拟器\n");
    printf("    输入传感器值, 观察压缩机频率和膨胀阀开度\n");
    print_separator();
    printf("\n  操作说明:\n");
    printf("    - 输入新的传感器值 (直接回车保持当前值)\n");
    printf("    - 输入 q 退出程序\n");
    printf("    - 每次输入后自动计算频率调节(1次)和阀门调节(1次)\n");
    printf("\n");

    while (1) {
        sim_seconds++;
        print_separator();
        printf("  >>>  第 %d 轮模拟  <<<\n", sim_seconds);
        print_separator();
        printf("\n  == 输入传感器数值 (输入 q 退出) ==\n\n");

        /* 读取各传感器值 */
        printf("  --- 温度传感器 ---\n");
        s.cabinet_temp   = read_float("柜温 (C)          ", s.cabinet_temp);
        s.set_temp       = read_float("设定温度 (C)      ", s.set_temp);
        s.evap_temp      = read_float("蒸发器温度 (C)    ", s.evap_temp);
        s.suction_temp   = read_float("吸气温度 (C)      ", s.suction_temp);
        s.discharge_temp = read_float("排气温度 (C)      ", s.discharge_temp);
        s.sat_temp_low   = read_float("低压饱和温度 (C)  ", s.sat_temp_low);

        printf("\n  --- 压力传感器 ---\n");
        s.discharge_pres = read_float("排气压力 (bar)    ", s.discharge_pres);
        s.suction_pres   = read_float("吸气压力 (bar)    ", s.suction_pres);

        /* 也允许直接修改当前输出值(模拟连续运行) */
        printf("\n  --- 当前输出状态 (可修改模拟连续运行) ---\n");
        s.comp_freq      = read_float("压缩机频率 (Hz)   ", s.comp_freq);
        s.exv_opening    = read_float("膨胀阀开度 (步)   ", s.exv_opening);

        /* 执行控制逻辑 */
        sim_freq_adjust(&s);
        sim_exv_adjust(&s);
        sim_alarm_check(&s);

        /* 打印结果 */
        printf("\n");
        print_state(&s);

        /* 询问是否继续 */
        printf("\n  按回车继续下一轮, 输入 q 退出: ");
        fflush(stdout);
        if (fgets(buf, sizeof(buf), stdin) == NULL) break;
        if (buf[0] == 'q' || buf[0] == 'Q') break;
        printf("\n");
    }

    printf("\n  模拟结束。\n\n");
    return 0;
}
