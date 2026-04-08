/*
 * sim_test.c - 模拟器交互测试 (增强版)
 *
 * ===================== 功能概览 =====================
 *
 *   1. 上电启动流程模拟:
 *      - EXV归零 (关阀550步)
 *      - 与变频板建立通信 (RS485初始化)
 *      - 发送启动命令 (0x01, 120Hz) → 看16字节数据包
 *      - EXV开到初始开度 (250步)
 *      - 热机等待 (C20 = 10秒)
 *
 *   2. 运行中控制:
 *      - 压缩机频率PID调节 (每1秒)
 *      - 膨胀阀开度调节 (每30秒)
 *      - 每次调节都能看到发给变频板的16字节数据包
 *      - 能看到EXV步进电机的4相驱动模式
 *
 *   3. 停机:
 *      - 发送停机命令 (0x00) → 看停机数据包
 *      - EXV关阀
 *
 * ===================== 用法 =====================
 *
 *   1. main.c 里打开 #define SIM_MODE
 *   2. 编译, Ctrl+F5 进Debug
 *   3. 在 SIM_RunTest() 的 while(1) 里设断点
 *   4. Watch窗口添加所有 sim_in_xxx 和 sim_out_xxx 变量
 *   5. 修改 sim_in_phase 切换阶段:
 *        0 = 上电启动 (自动走完启动流程)
 *        1 = 正常运行 (改输入值看控制结果)
 *        2 = 停机 (看停机数据包)
 *   6. 按F5继续, 观察输出变化
 *
 * ===================== Watch窗口推荐 =====================
 *
 *   输入:
 *     sim_in_phase, sim_in_cabinet_temp, sim_in_set_temp,
 *     sim_in_evap_temp, sim_in_superheat,
 *     sim_in_exv_opening, sim_in_comp_freq
 *
 *   控制输出:
 *     sim_out_delta_t, sim_out_freq_step, sim_out_new_freq,
 *     sim_out_new_exv, sim_out_exv_action, sim_out_alarm_edt
 *
 *   数据包 (重点!):
 *     sim_out_packet (展开看16个字节)
 *     sim_out_pkt_cmd, sim_out_pkt_freq_hz,
 *     sim_out_pkt_desc (包描述)
 *
 *   启动流程:
 *     sim_out_startup_step, sim_out_startup_desc
 *
 *   EXV步进电机:
 *     sim_out_exv_pos, sim_out_exv_dir,
 *     sim_out_exv_phase (展开看4相)
 *
 *   通信状态:
 *     sim_out_inv_echo_ok, sim_out_inv_fail_reason,
 *     sim_out_inv_comm_ok
 */
#include "sim_test.h"
#include "sys_config.h"
#include <string.h>

/* ====================================================================
 *  EXV 步进电机相位表 (与 bsp_exv.c 完全一致)
 *  4相2相全步驱动, 每步同时通电2个线圈
 *
 *  索引: {PM0A(B-), PM0B(A-), PM0C(B+), PM0D(A+)}
 * ==================================================================== */
static const unsigned char PHASE_TABLE[4][4] = {
    /* B-  A-  B+  A+ */
    {  0,  0,  1,  1 },  /* Step 0: A+ & B+ 通电 */
    {  0,  1,  1,  0 },  /* Step 1: A- & B+ 通电 */
    {  1,  1,  0,  0 },  /* Step 2: A- & B- 通电 */
    {  1,  0,  0,  1 },  /* Step 3: A+ & B- 通电 */
};

/* ====================================================================
 *  >>> 输入变量 (在Watch窗口修改) <<<
 * ==================================================================== */

/* --- 阶段控制 --- */
int sim_in_phase         = 0;        /* 0=上电启动, 1=正常运行, 2=停机 */

/* --- 传感器模拟值 (运行阶段用) --- */
float sim_in_cabinet_temp  = 10.0f;  /* 柜温 (°C)                      */
float sim_in_set_temp      = -10.0f; /* 设定温度 (°C)                   */
float sim_in_evap_temp     = -2.0f;  /* 蒸发温度 (°C)                   */
float sim_in_superheat     = 8.0f;   /* 过热度 (°C)                     */
float sim_in_exv_opening   = 250.0f; /* 膨胀阀当前开度 (0~500步)        */
float sim_in_comp_freq     = 120.0f; /* 压缩机当前频率 (Hz)             */

/* --- 通信模拟 (模拟变频板是否应答) --- */
int sim_in_inv_echo_ok     = 1;      /* 1=模拟echo成功, 0=模拟echo失败  */

/* ====================================================================
 *  >>> 输出变量 — 控制算法结果 <<<
 * ==================================================================== */
float sim_out_delta_t;       /* 温差 = 柜温 - 设定温度              */
int   sim_out_freq_step;     /* 频率步长: +3/+2/+1/0/-1/-2/-3 Hz/s  */
float sim_out_new_freq;      /* 调整后的频率                        */

float sim_out_delta_tcz;     /* 传热温差 = 柜温 - 蒸发温度          */
float sim_out_tcz_err;       /* 传热温差偏差 = 传热温差 - 6.5       */
float sim_out_new_exv;       /* 调整后的膨胀阀开度                  */
int   sim_out_exv_action;    /* 0=不动 1=温差调整 2=过热度报警调整   */
int   sim_out_alarm_edt;     /* 1=过热度过低报警 (EDT)              */

/* ====================================================================
 *  >>> 输出变量 — 变频板16字节数据包 (核心!) <<<
 *
 *  在Watch窗口展开 sim_out_packet 可以看到每个字节:
 *    [0]  = 0x55 帧头
 *    [1]  = CMD  命令 (0x00停/0x01启/0x02调频)
 *    [2]  = 频率低字节
 *    [3]  = 频率高字节
 *    [4~14] = 0x00 保留
 *    [15] = 0x56 帧尾
 * ==================================================================== */
unsigned char sim_out_packet[16];     /* 完整16字节帧 (展开看!)        */
unsigned char sim_out_pkt_cmd;        /* 命令字节单独显示              */
unsigned int  sim_out_pkt_freq_hz;    /* 频率值单独显示 (Hz)           */
unsigned char sim_out_pkt_freq_lo;    /* 频率低字节                    */
unsigned char sim_out_pkt_freq_hi;    /* 频率高字节                    */
int  sim_out_pkt_valid;               /* 1=本轮有新包, 0=无            */
char sim_out_pkt_desc[40];            /* 包描述, 如 "START 120Hz"      */

/* ====================================================================
 *  >>> 输出变量 — 变频板通信状态 <<<
 * ==================================================================== */
int  sim_out_inv_echo_ok;     /* echo校验结果: 1=通过, 0=失败       */
int  sim_out_inv_fail_reason; /* 失败原因: 0=OK, 1=超时, 2=不一致   */
int  sim_out_inv_comm_ok;     /* 通信是否正常                       */
int  sim_out_inv_total_sent;  /* 累计发送包数                       */
int  sim_out_inv_total_fail;  /* 累计失败次数                       */

/* ====================================================================
 *  >>> 输出变量 — EXV步进电机状态 <<<
 * ==================================================================== */
unsigned int  sim_out_exv_pos;       /* 当前位置 (0~500步)             */
int           sim_out_exv_dir;       /* 方向: 0=关(CW), 1=开(CCW)     */
unsigned int  sim_out_exv_steps_moved; /* 本次移动步数                 */
unsigned char sim_out_exv_phase[4];  /* 当前相位 {B-, A-, B+, A+}     */
int           sim_out_exv_phase_idx; /* 相位索引 (0~3)                */
char          sim_out_exv_desc[40];  /* EXV动作描述                   */

/* ====================================================================
 *  >>> 输出变量 — 启动流程 <<<
 * ==================================================================== */
int  sim_out_startup_step;    /* 当前启动步骤 (0~6)                  */
char sim_out_startup_desc[48]; /* 步骤描述                           */
int  sim_out_warmup_remain;   /* 热机剩余秒数                       */

/* 循环计数 */
int  sim_loop_count = 0;

/* ====================================================================
 *  内部辅助函数
 * ==================================================================== */

/* 组装16字节下行帧 (与 bsp_inverter.c BuildFrame 完全一致) */
static void sim_build_packet(unsigned char cmd, unsigned int freq_hz)
{
    memset(sim_out_packet, 0x00, 16);

    sim_out_packet[0]  = INV_FRAME_HEAD_VAL;    /* 0x55 帧头 */
    sim_out_packet[1]  = cmd;                    /* 命令 */
    sim_out_packet[2]  = (unsigned char)(freq_hz & 0xFF);        /* 频率低字节 */
    sim_out_packet[3]  = (unsigned char)((freq_hz >> 8) & 0xFF); /* 频率高字节 */
    /* [4~14] 保留, 已被memset清零 */
    sim_out_packet[15] = INV_FRAME_TAIL_VAL;    /* 0x56 帧尾 */

    /* 单独显示关键字段, 方便Watch窗口直接看 */
    sim_out_pkt_cmd     = cmd;
    sim_out_pkt_freq_hz = freq_hz;
    sim_out_pkt_freq_lo = sim_out_packet[2];
    sim_out_pkt_freq_hi = sim_out_packet[3];
    sim_out_pkt_valid   = 1;
}

/* 模拟发送并校验echo */
static void sim_send_and_verify(void)
{
    sim_out_inv_total_sent++;

    if (sim_in_inv_echo_ok) {
        /* 模拟echo成功 */
        sim_out_inv_echo_ok     = 1;
        sim_out_inv_fail_reason = 0;
        sim_out_inv_comm_ok     = 1;
    } else {
        /* 模拟echo失败 (超时) */
        sim_out_inv_echo_ok     = 0;
        sim_out_inv_fail_reason = 1;  /* 1=超时无回应 */
        sim_out_inv_comm_ok     = 0;
        sim_out_inv_total_fail++;
    }
}

/* 模拟EXV移动到目标位置 */
static void sim_exv_move_to(unsigned int target, const char *desc)
{
    if (target > 500) target = 500;

    if (target > sim_out_exv_pos) {
        sim_out_exv_dir = 1;  /* 开阀 */
        sim_out_exv_steps_moved = target - sim_out_exv_pos;
    } else if (target < sim_out_exv_pos) {
        sim_out_exv_dir = 0;  /* 关阀 */
        sim_out_exv_steps_moved = sim_out_exv_pos - target;
    } else {
        sim_out_exv_dir = 0;
        sim_out_exv_steps_moved = 0;
    }

    sim_out_exv_pos = target;

    /* 计算当前相位 (模拟步进电机停止时的线圈状态) */
    sim_out_exv_phase_idx = (int)(target & 0x03);
    sim_out_exv_phase[0] = PHASE_TABLE[sim_out_exv_phase_idx][0]; /* B- */
    sim_out_exv_phase[1] = PHASE_TABLE[sim_out_exv_phase_idx][1]; /* A- */
    sim_out_exv_phase[2] = PHASE_TABLE[sim_out_exv_phase_idx][2]; /* B+ */
    sim_out_exv_phase[3] = PHASE_TABLE[sim_out_exv_phase_idx][3]; /* A+ */

    /* 复制描述 */
    {
        int i;
        for (i = 0; desc[i] && i < 39; i++)
            sim_out_exv_desc[i] = desc[i];
        sim_out_exv_desc[i] = '\0';
    }
}

/* 安全复制字符串 (避免依赖 strncpy) */
static void sim_strcpy(char *dst, const char *src, int max_len)
{
    int i;
    for (i = 0; src[i] && i < max_len - 1; i++)
        dst[i] = src[i];
    dst[i] = '\0';
}

/* ====================================================================
 *  阶段0: 上电启动流程模拟
 *
 *  每按一次F5走一个子步骤, 完整模拟:
 *    INIT → EXV归零 → 变频板初始化 → 发启动包 → EXV初始开度 → 热机
 * ==================================================================== */
static void sim_phase_startup(void)
{
    switch (sim_out_startup_step) {

    case STARTUP_STEP_INIT:
        /* ---- 步骤0: 系统初始化 ---- */
        sim_strcpy(sim_out_startup_desc, "[0] SYS INIT: GPIO+UART+RTOS", 48);

        /* 清空所有状态 */
        sim_out_exv_pos = 500;  /* 假设上电时阀门位置未知, 设为满开 */
        sim_out_inv_total_sent = 0;
        sim_out_inv_total_fail = 0;
        sim_out_inv_comm_ok = 0;
        sim_out_warmup_remain = SET_WARMUP_C20;
        sim_out_pkt_valid = 0;
        memset(sim_out_packet, 0, 16);

        sim_out_startup_step = STARTUP_STEP_EXV_RESET;
        break;

    case STARTUP_STEP_EXV_RESET:
        /* ---- 步骤1: EXV归零 (关阀550步, 确保全关) ---- */
        sim_strcpy(sim_out_startup_desc, "[1] EXV RESET: close 550 steps", 48);
        sim_exv_move_to(0, "RESET->0 (550 steps CW)");

        sim_out_startup_step = STARTUP_STEP_INV_INIT;
        break;

    case STARTUP_STEP_INV_INIT:
        /* ---- 步骤2: 变频板通信初始化 ---- */
        sim_strcpy(sim_out_startup_desc, "[2] INV INIT: RS485 RX mode", 48);

        /* 模拟: RS485方向引脚设为接收, 禁用UART中断, 创建互斥锁 */
        sim_out_inv_echo_ok = 0;
        sim_out_inv_fail_reason = 0;
        sim_out_inv_comm_ok = 0;
        sim_out_pkt_valid = 0;

        sim_out_startup_step = STARTUP_STEP_INV_START;
        break;

    case STARTUP_STEP_INV_START:
        /* ---- 步骤3: 发送启动命令 → 看数据包! ---- */
        sim_strcpy(sim_out_startup_desc, "[3] INV START: send 0x01 120Hz", 48);

        /* 组装启动帧: CMD=0x01(启动), FREQ=120Hz */
        sim_build_packet(INV_CMD_START, 120);
        sim_strcpy(sim_out_pkt_desc, "START cmd=0x01 freq=120Hz", 40);

        /* 模拟发送+echo校验 */
        sim_send_and_verify();

        sim_out_startup_step = STARTUP_STEP_EXV_OPEN;
        break;

    case STARTUP_STEP_EXV_OPEN:
        /* ---- 步骤4: EXV开到初始开度 250步 (50%) ---- */
        sim_strcpy(sim_out_startup_desc, "[4] EXV OPEN: goto 250 steps", 48);
        sim_exv_move_to(250, "INIT open 0->250 (CCW)");
        sim_out_pkt_valid = 0;  /* 此步骤无新包 */

        sim_out_startup_step = STARTUP_STEP_WARMUP;
        break;

    case STARTUP_STEP_WARMUP:
        /* ---- 步骤5: 热机等待 ---- */
        if (sim_out_warmup_remain > 0) {
            sim_strcpy(sim_out_startup_desc, "[5] WARMUP: waiting C20...", 48);
            sim_out_warmup_remain--;
            sim_out_pkt_valid = 0;
            /* 停在此步骤直到倒计时结束 (每次F5减1秒) */
        }

        if (sim_out_warmup_remain <= 0) {
            sim_out_startup_step = STARTUP_STEP_DONE;
        }
        break;

    case STARTUP_STEP_DONE:
        /* ---- 步骤6: 启动完成 ---- */
        sim_strcpy(sim_out_startup_desc, "[6] STARTUP DONE -> RUNNING", 48);
        sim_out_pkt_valid = 0;

        /* 自动切换到运行阶段 */
        sim_in_phase = SIM_PHASE_RUNNING;
        sim_in_comp_freq = 120.0f;
        sim_in_exv_opening = 250.0f;
        break;
    }
}

/* ====================================================================
 *  阶段1: 正常运行控制 (频率调节 + 膨胀阀调节 + 数据包生成)
 * ==================================================================== */
static void sim_phase_running(void)
{
    unsigned int freq_u16;

    /* ============ 频率调节 (每1秒执行) ============ */
    sim_out_delta_t = sim_in_cabinet_temp - sim_in_set_temp;

    sim_out_freq_step = 0;

    if (sim_out_delta_t >= 5.0f) {
        sim_out_freq_step = 3;           /* 温差>=5°C: +3Hz/s  */
    } else if (sim_out_delta_t >= 2.0f) {
        sim_out_freq_step = 2;           /* 温差2~5°C: +2Hz/s  */
    } else if (sim_out_delta_t > 0.0f) {
        sim_out_freq_step = 1;           /* 温差0~2°C: +1Hz/s  */
    } else if (sim_out_delta_t <= -5.0f) {
        sim_out_freq_step = -3;          /* 温差<=-5°C: -3Hz/s */
    } else if (sim_out_delta_t <= -2.0f) {
        sim_out_freq_step = -2;          /* 温差-5~-2°C: -2Hz/s */
    } else if (sim_out_delta_t < 0.0f) {
        sim_out_freq_step = -1;          /* 温差-2~0°C: -1Hz/s  */
    }

    sim_out_new_freq = sim_in_comp_freq + sim_out_freq_step;
    if (sim_out_new_freq > 320.0f) sim_out_new_freq = 320.0f;
    if (sim_out_new_freq < 120.0f) sim_out_new_freq = 120.0f;

    /* ---- 生成调频数据包 ---- */
    freq_u16 = (unsigned int)sim_out_new_freq;
    sim_build_packet(INV_CMD_SET_FREQ, freq_u16);

    /* 生成包描述 */
    if (sim_out_freq_step > 0) {
        sim_strcpy(sim_out_pkt_desc, "FREQ_ADJ: speed UP", 40);
    } else if (sim_out_freq_step < 0) {
        sim_strcpy(sim_out_pkt_desc, "FREQ_ADJ: speed DOWN", 40);
    } else {
        sim_strcpy(sim_out_pkt_desc, "FREQ_ADJ: hold steady", 40);
    }

    /* 模拟发送+echo */
    sim_send_and_verify();

    /* ============ 膨胀阀调节 (每30秒执行) ============ */
    sim_out_delta_tcz = sim_in_cabinet_temp - sim_in_evap_temp;
    sim_out_tcz_err = 0;
    sim_out_new_exv = sim_in_exv_opening;
    sim_out_exv_action = 0;
    sim_out_alarm_edt = 0;

    if (sim_out_delta_tcz < 5.0f || sim_out_delta_tcz > 8.0f) {
        /* 传热温差不在[5.0, 8.0]范围 → 调整阀门 */
        sim_out_tcz_err = sim_out_delta_tcz - SET_HT_DIFF_TARGET;
        sim_out_new_exv = sim_in_exv_opening - SET_PID_ALPHA1 * sim_out_tcz_err;
        sim_out_exv_action = 1;
    } else {
        /* 传热温差OK, 看过热度 */
        if (sim_in_superheat < SET_SH_MIN_LOW) {
            /* 过热度过低 → EDT报警, 关小阀门 */
            sim_out_new_exv = sim_in_exv_opening - SET_PID_ALPHA2 * sim_in_superheat;
            sim_out_exv_action = 2;
            sim_out_alarm_edt = 1;
        } else {
            /* 全部正常, 不调整 */
            sim_out_exv_action = 0;
        }
    }

    if (sim_out_new_exv > 500.0f) sim_out_new_exv = 500.0f;
    if (sim_out_new_exv < 0.0f)   sim_out_new_exv = 0.0f;

    /* ---- 更新EXV步进电机状态 ---- */
    {
        unsigned int target = (unsigned int)sim_out_new_exv;
        if (sim_out_exv_action == 0) {
            sim_exv_move_to(target, "EXV: no change");
        } else if (sim_out_exv_action == 1) {
            sim_exv_move_to(target, "EXV: HT_DIFF adjust");
        } else {
            sim_exv_move_to(target, "EXV: EDT alarm close");
        }
    }
}

/* ====================================================================
 *  阶段2: 停机
 * ==================================================================== */
static void sim_phase_stop(void)
{
    /* 发送停机命令 */
    sim_build_packet(INV_CMD_STOP, 0);
    sim_strcpy(sim_out_pkt_desc, "STOP cmd=0x00 freq=0", 40);
    sim_send_and_verify();

    /* EXV关阀 */
    sim_exv_move_to(0, "STOP: close valve");

    /* 清除运行状态 */
    sim_out_new_freq = 0;
    sim_out_freq_step = 0;
    sim_out_delta_t = 0;
}

/* ====================================================================
 *  主入口
 * ==================================================================== */
void SIM_RunTest(void)
{
    /* 初始化启动步骤 */
    sim_out_startup_step = STARTUP_STEP_INIT;
    sim_out_exv_pos = 0;
    sim_out_exv_phase_idx = 0;
    memset(sim_out_exv_phase, 0, 4);
    memset(sim_out_packet, 0, 16);
    sim_out_inv_total_sent = 0;
    sim_out_inv_total_fail = 0;

    while (1)  /* <<< 在这行下面设断点 >>> */
    {
        sim_loop_count++;

        switch (sim_in_phase) {

        case SIM_PHASE_STARTUP:
            sim_phase_startup();
            break;

        case SIM_PHASE_RUNNING:
            sim_phase_running();
            break;

        case SIM_PHASE_STOP:
            sim_phase_stop();
            break;

        default:
            sim_in_phase = SIM_PHASE_STARTUP;
            break;
        }

        /* >>>>>> 断点设在这里 <<<<<<
         *
         * 每次停下来:
         *
         * 【启动阶段 sim_in_phase=0】
         *   看 sim_out_startup_step 和 sim_out_startup_desc
         *   步骤3时展开 sim_out_packet 看16字节启动包:
         *     [0]=0x55  [1]=0x01  [2]=0x78  [3]=0x00  ...  [15]=0x56
         *     含义: 帧头    启动    120低字节  120高字节       帧尾
         *
         * 【运行阶段 sim_in_phase=1】
         *   1. 改 sim_in_cabinet_temp 等输入值
         *   2. 看 sim_out_new_freq (新频率) 和 sim_out_new_exv (新开度)
         *   3. 展开 sim_out_packet 看调频包:
         *      [0]=0x55  [1]=0x02  [2]=freq_lo  [3]=freq_hi  ...  [15]=0x56
         *   4. 看 sim_out_exv_phase 了解步进电机线圈状态
         *   5. 改 sim_in_inv_echo_ok=0 模拟通信失败
         *
         * 【停机 sim_in_phase=2】
         *   看停机包: [1]=0x00, [2]=0x00, [3]=0x00
         *
         * 按F5继续下一轮
         */
    }
}
