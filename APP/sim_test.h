#ifndef SIM_TEST_H
#define SIM_TEST_H

#include <stdint.h>

/* ===================================================================
 *  sim_test.h - 模拟器交互测试 (增强版)
 *
 *  功能:
 *    1. 压缩机频率调节 + 膨胀阀开度调节 (纯算法)
 *    2. 上电启动流程模拟 (EXV归零 → 变频板握手 → 初始开度)
 *    3. 变频板16字节数据包可视化 (在Watch窗口看每个字节)
 *    4. EXV步进电机相位模式可视化
 *
 *  不依赖任何硬件, 纯逻辑验证
 * =================================================================== */

/* --- 模拟阶段 (sim_in_phase) --- */
#define SIM_PHASE_STARTUP   0   /* 上电启动流程 */
#define SIM_PHASE_RUNNING   1   /* 正常运行控制 */
#define SIM_PHASE_STOP      2   /* 停机 */

/* --- 启动子步骤 (sim_out_startup_step) --- */
#define STARTUP_STEP_INIT       0   /* 系统初始化 */
#define STARTUP_STEP_EXV_RESET  1   /* EXV归零(关阀550步) */
#define STARTUP_STEP_INV_INIT   2   /* 变频板通信初始化 */
#define STARTUP_STEP_INV_START  3   /* 发送启动命令(0x01, 120Hz) */
#define STARTUP_STEP_EXV_OPEN   4   /* EXV开到初始开度(250步) */
#define STARTUP_STEP_WARMUP     5   /* 热机等待(C20=10秒) */
#define STARTUP_STEP_DONE       6   /* 启动完成, 进入运行 */

/* --- 变频板命令 --- */
#define INV_CMD_STOP        0x00
#define INV_CMD_START       0x01
#define INV_CMD_SET_FREQ    0x02

/* --- 帧格式 --- */
#define INV_FRAME_HEAD_VAL  0x55
#define INV_FRAME_TAIL_VAL  0x56
#define INV_PKT_LEN         16

/* 模拟器测试入口 */
void SIM_RunTest(void);

#endif /* SIM_TEST_H */
