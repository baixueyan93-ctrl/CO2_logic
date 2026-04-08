/*
 * sim_test.c - 模拟器纯逻辑测试
 *
 * 用法:
 *   1. main.c 最前面 #define SIM_MODE
 *   2. 编译, 进入Debug (Ctrl+F5)
 *   3. 在 SIM_RunTest() 里设断点
 *   4. F10 单步, Watch窗口看变量变化
 *   5. 测试完把 #define SIM_MODE 注释掉, 恢复正常编译
 */
#include "sim_test.h"
#include "sys_config.h"

/* ============================================
 * 模拟用的数据结构 (和真实代码一样的字段)
 * ============================================ */
typedef struct {
    float cabinet_temp;     /* 柜温 */
    float evap_temp;        /* 蒸发温度 */
    float comp_in_temp;     /* 压缩机吸气温度 */
    float superheat;        /* 过热度 = 吸气温度 - 蒸发温度 */
    float exv_opening;      /* 当前膨胀阀开度 (0~500步) */
    float comp_freq;        /* 当前压缩机频率 (Hz) */
    float set_temp;         /* 设定温度 */
} SimSensor_t;

/* 测试结果, Watch窗口看这些变量 */
static float result_new_freq;      /* 频率调整后的值 */
static float result_new_exv;       /* 膨胀阀调整后的值 */
static float result_delta_t;       /* 温差 = 柜温 - 设定温度 */
static float result_delta_tcz;     /* 传热温差 = 柜温 - 蒸发温度 */
static int   result_freq_step;     /* 频率调整步长 (+3/+2/+1/0/-1/-2/-3) */
static int   result_exv_action;    /* 0=不调 1=温差调整 2=过热度调整 */
static int   result_alarm;         /* 0=无报警 1=过热度过低EDT */

/* ============================================
 *  测试1: 频率调节逻辑
 *  和 task_freq_exv.c 的 FreqExv_FreqAdjust() 一样的算法
 * ============================================ */
static void SIM_TestFreqAdjust(SimSensor_t *s)
{
    float delta_t = s->cabinet_temp - s->set_temp;
    float current_freq = s->comp_freq;
    float new_freq = current_freq;

    result_delta_t = delta_t;

    /* 和真实代码一样的分档逻辑 */
    if (delta_t >= SET_DT_MAX) {
        /* 温差>=5度, 快速升频 +3Hz/s */
        new_freq = current_freq + 3.0f;
        result_freq_step = 3;
    } else if (delta_t >= 2.0f) {
        /* 温差2~5度, 中速升频 +2Hz/s */
        new_freq = current_freq + 2.0f;
        result_freq_step = 2;
    } else if (delta_t > 0.0f) {
        /* 温差0~2度, 慢速升频 +1Hz/s */
        new_freq = current_freq + 1.0f;
        result_freq_step = 1;
    } else if (delta_t <= -SET_DT_MAX) {
        /* 温差<=-5度, 快速降频 -3Hz/s */
        new_freq = current_freq - 3.0f;
        result_freq_step = -3;
    } else if (delta_t <= -2.0f) {
        /* 温差-5~-2度, 中速降频 -2Hz/s */
        new_freq = current_freq - 2.0f;
        result_freq_step = -2;
    } else if (delta_t < 0.0f) {
        /* 温差-2~0度, 慢速降频 -1Hz/s */
        new_freq = current_freq - 1.0f;
        result_freq_step = -1;
    } else {
        result_freq_step = 0;
    }

    /* 上下限保护 */
    if (new_freq > SET_FREQ_MAX) new_freq = SET_FREQ_MAX;  /* 320Hz */
    if (new_freq < SET_FREQ_MIN) new_freq = SET_FREQ_MIN;  /* 120Hz */

    result_new_freq = new_freq;
    s->comp_freq = new_freq;  /* 更新, 供下次调用 */
}

/* ============================================
 *  测试2: 膨胀阀PID调节逻辑
 *  和 task_freq_exv.c 的 FreqExv_ExvAdjust() 一样的算法
 * ============================================ */
static void SIM_TestExvAdjust(SimSensor_t *s)
{
    float kp = s->exv_opening;
    float delta_tcz = s->cabinet_temp - s->evap_temp;
    float superheat = s->superheat;

    result_delta_tcz = delta_tcz;
    result_alarm = 0;
    result_exv_action = 0;

    /* 第1步: 传热温差是否在目标范围 [5.0, 8.0] */
    float tcz_low  = SET_HT_DIFF_TARGET - SET_HT_DIFF_TOL;  /* 5.0 */
    float tcz_high = SET_HT_DIFF_TARGET + SET_HT_DIFF_TOL;  /* 8.0 */

    if (delta_tcz < tcz_low || delta_tcz > tcz_high) {
        /* 温差偏离目标范围 -> 调整阀门 */
        float tcz_err = delta_tcz - SET_HT_DIFF_TARGET;
        kp = kp - SET_PID_ALPHA1 * tcz_err;  /* alpha1=0.5 */
        result_exv_action = 1;  /* 温差调整 */
    } else {
        /* 温差OK, 检查过热度 */
        if (superheat < SET_SH_MIN_LOW) {
            /* 过热度过低 < 5.0度, EDT报警 */
            kp = kp - SET_PID_ALPHA2 * superheat;  /* alpha2=0.8 */
            result_exv_action = 2;  /* 过热度调整 */
            result_alarm = 1;
        } else {
            /* 全部正常, 不调整 */
            result_exv_action = 0;
        }
    }

    /* 开度上下限保护 */
    if (kp > 500.0f) kp = 500.0f;
    if (kp < 0.0f)   kp = 0.0f;

    result_new_exv = kp;
    s->exv_opening = kp;  /* 更新 */
}

/* ============================================
 *  主测试入口 - 在这里设断点, F10单步
 * ============================================ */
void SIM_RunTest(void)
{
    SimSensor_t sensor;

    /* ========== 场景1: 正常制冷, 柜温偏高 ========== */
    /*  柜温10度, 设定-10度, 蒸发-2度, 过热度8度       */
    /*  预期: 频率升+3Hz, 膨胀阀因温差偏大而关小       */
    sensor.cabinet_temp  = 10.0f;    /* <- 在Watch里改这些值试不同场景 */
    sensor.set_temp      = -10.0f;
    sensor.evap_temp     = -2.0f;
    sensor.comp_in_temp  = 6.0f;
    sensor.superheat     = 8.0f;
    sensor.exv_opening   = 250.0f;
    sensor.comp_freq     = 120.0f;

    SIM_TestFreqAdjust(&sensor);    /* <- F10走到这里, 看result_new_freq */
    SIM_TestExvAdjust(&sensor);     /* <- F10走到这里, 看result_new_exv  */

    /* ========== 场景2: 接近目标温度 ========== */
    /*  柜温-9度, 设定-10度, 温差只有1度             */
    /*  预期: 频率仅升+1Hz, 膨胀阀看温差和过热度     */
    sensor.cabinet_temp  = -9.0f;
    sensor.set_temp      = -10.0f;
    sensor.evap_temp     = -15.0f;
    sensor.comp_in_temp  = -9.0f;
    sensor.superheat     = 6.0f;
    sensor.exv_opening   = 250.0f;
    sensor.comp_freq     = 200.0f;

    SIM_TestFreqAdjust(&sensor);
    SIM_TestExvAdjust(&sensor);

    /* ========== 场景3: 柜温已低于设定, 需要降频 ========== */
    /*  柜温-13度, 设定-10度, 温差-3度                     */
    /*  预期: 频率降-2Hz                                    */
    sensor.cabinet_temp  = -13.0f;
    sensor.set_temp      = -10.0f;
    sensor.evap_temp     = -20.0f;
    sensor.comp_in_temp  = -14.0f;
    sensor.superheat     = 6.0f;
    sensor.exv_opening   = 250.0f;
    sensor.comp_freq     = 200.0f;

    SIM_TestFreqAdjust(&sensor);
    SIM_TestExvAdjust(&sensor);

    /* ========== 场景4: 过热度过低, EDT报警 ========== */
    /*  温差正常(6.5范围内), 但过热度只有3度            */
    /*  预期: EDT报警, 膨胀阀关小                       */
    sensor.cabinet_temp  = -4.0f;
    sensor.set_temp      = -10.0f;
    sensor.evap_temp     = -10.5f;
    sensor.comp_in_temp  = -7.5f;
    sensor.superheat     = 3.0f;
    sensor.exv_opening   = 250.0f;
    sensor.comp_freq     = 180.0f;

    SIM_TestFreqAdjust(&sensor);
    SIM_TestExvAdjust(&sensor);

    /* ========== 场景5: 全部正常, 不调整 ========== */
    /*  温差6.5度(目标值), 过热度8度(正常)            */
    /*  预期: 膨胀阀不动, result_exv_action=0        */
    sensor.cabinet_temp  = -3.5f;
    sensor.set_temp      = -10.0f;
    sensor.evap_temp     = -10.0f;
    sensor.comp_in_temp  = -2.0f;
    sensor.superheat     = 8.0f;
    sensor.exv_opening   = 250.0f;
    sensor.comp_freq     = 180.0f;

    SIM_TestFreqAdjust(&sensor);
    SIM_TestExvAdjust(&sensor);

    /* ===== 测试结束, 程序停在这里 ===== */
    /* 在Watch窗口查看所有 result_ 开头的变量 */
    while(1) {}
}
