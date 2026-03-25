#ifndef BSP_EXV_H
#define BSP_EXV_H

#include "main.h"
#include <stdint.h>

/* ===================================================================
 *  鹭宫 VKV 电子膨胀阀 步进电机驱动 (DC12V, 单极性4相)
 *
 *  硬件连接 (V13 原理图 XH-5AW EXV0):
 *    PM0A (B-) = PD11  (PM01)
 *    PM0B (A-) = PD10  (PM02)
 *    PM0C (B+) = PD9   (PM03)
 *    PM0D (A+) = PD8   (PM04)
 *
 *  信号链: MCU GPIO -> TLP291-4 光耦 (U10) -> ULN2803A (U14) -> EXV0
 *  逻辑: GPIO HIGH = 线圈通电
 * =================================================================== */

/* --- 引脚定义 --- */
#define EXV0_PM0A_PORT   GPIOD          /* B-  */
#define EXV0_PM0A_PIN    GPIO_PIN_11
#define EXV0_PM0B_PORT   GPIOD          /* A-  */
#define EXV0_PM0B_PIN    GPIO_PIN_10
#define EXV0_PM0C_PORT   GPIOD          /* B+  */
#define EXV0_PM0C_PIN    GPIO_PIN_9
#define EXV0_PM0D_PORT   GPIOD          /* A+  */
#define EXV0_PM0D_PIN    GPIO_PIN_8

/* --- 阀门参数 --- */
#define EXV_TOTAL_STEPS     500     /* VKV 全行程步数              */
#define EXV_INIT_CLOSE_STEPS 550    /* 初始化关阀步数(多走50步确保全关) */
#define EXV_STEP_DELAY_MS    20     /* 每步延时 (ms), 约50 PPS     */

/* --- 方向 --- */
typedef enum {
    EXV_DIR_CLOSE = 0,   /* 关阀 (CW)  */
    EXV_DIR_OPEN  = 1    /* 开阀 (CCW) */
} EXV_Direction_t;

/* --- 接口函数 --- */
void     BSP_EXV_Init(void);
void     BSP_EXV_DeEnergize(void);
void     BSP_EXV_Step(EXV_Direction_t dir, uint16_t steps, uint16_t delay_ms);
uint16_t BSP_EXV_GetPosition(void);
void     BSP_EXV_SetPosition(uint16_t target_steps, uint16_t delay_ms);
void     BSP_EXV_ResetToZero(void);

#endif /* BSP_EXV_H */
