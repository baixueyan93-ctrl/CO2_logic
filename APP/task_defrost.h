#ifndef TASK_DEFROST_H
#define TASK_DEFROST_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * 除霜流程 (逻辑图2) — FreeRTOS 任务
 *
 * 包含子逻辑:
 *   1. 除霜主程序      Defrost_MainProcess()
 *   2. 加热子程序      Defrost_HeatSubroutine()
 *
 * 除霜三阶段状态机:
 *   [待机] → 间隔到时 → [加热] → 超时/温度达标 → [滴水] → 超时 → [待机]
 *
 * 后续扩展: 作为6大逻辑块之一, 独立FreeRTOS任务
 * =========================================================================== */

/* --- 任务入口 --- */
void Task_Defrost_Process(void const *argument);

/* --- 子逻辑 (内部调用, 也可单独测试) --- */
void Defrost_MainProcess(void);        /* 除霜主程序   */
void Defrost_HeatSubroutine(void);     /* 加热子程序   */

/* ===========================================================================
 * 硬件接口定义 (待确认后填入实际引脚)
 *
 * 除霜加热器:
 *   TODO: 确认除霜加热器继电器GPIO引脚
 * =========================================================================== */
#define DEF_HEATER_GPIO_PORT    GPIOD           /* 待确认 */
#define DEF_HEATER_GPIO_PIN     GPIO_PIN_1      /* 待确认 */

#endif /* TASK_DEFROST_H */
