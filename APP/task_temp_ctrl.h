#ifndef TASK_TEMP_CTRL_H
#define TASK_TEMP_CTRL_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 温度控制流程 (逻辑图1) — FreeRTOS 任务
 *
 * 包含子逻辑 (来自逻辑图1的5个子流程图):
 *   1. 停机异常逻辑告警        TempCtrl_ShutdownAlarm()
 *   2. 压缩机开机逻辑          TempCtrl_CompressorStart()
 *   3. 油壳加热逻辑            TempCtrl_OilHeatControl()
 *   4. 告警(温度压力)处理逻辑  TempCtrl_AlarmProcess()
 *
 * 后续扩展: 6大逻辑块各为一个FreeRTOS任务
 *   任务1: 温度控制流程  (本文件)
 *   任务2: 除霜流程
 *   任务3: 风机流程
 *   任务4: 变频控制和膨胀阀流程
 *   任务5: 定时中断服务流程
 *   任务6: 冷凝风机流程
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_TempCtrl_Process(void const *argument);

/* --- 子逻辑 (内部调用, 也可单独测试) --- */
void TempCtrl_ShutdownAlarm(void);     /* 停机异常逻辑告警           */
void TempCtrl_CompressorStart(void);   /* 压缩机开机逻辑             */
void TempCtrl_OilHeatControl(void);    /* 油壳加热逻辑               */
void TempCtrl_AlarmProcess(void);      /* 告警(温度压力)处理逻辑     */

/* ===========================================================================
 * 硬件接口定义 (待确认后填入实际引脚)
 *
 * V13.pdf 参考:
 *   PB1 (VSININ) — 电源电压VDC采集 (ADC输入)
 *   K2  (PSD0D)  — 滑油热丝/油壳加热继电器输出
 * =========================================================================== */

/* --- VDC电压采集 (PB1/VSININ) ---
 * TODO: 确认ADC通道号, 当前通过 VAR_VDC_VOLTAGE 从sys_state读取
 *       采集驱动应在 task_adc 中完成, 本模块只读取结果
 */

/* --- 油壳加热继电器 K2 (PSD0D) ---
 * TODO: 确认实际GPIO引脚, 暂定义占位宏
 */
#define OIL_HEATER_GPIO_PORT    GPIOD           /* 待确认 */
#define OIL_HEATER_GPIO_PIN     GPIO_PIN_0      /* 待确认: PSD0D */

/* --- 压缩机控制接口 ---
 * TODO: 确认实际控制方式 (变频器通信 / GPIO)
 */

#endif /* TASK_TEMP_CTRL_H */
