#ifndef BSP_EXV_H
#define BSP_EXV_H

#include "main.h"
#include <stdint.h>

/* ===================================================================
 *  �ع� VKV �������ͷ� ����������� (DC12V, ������4��)
 *
 *  Ӳ������ (V13 ԭ��ͼ XH-5AW EXV0):
 *    PM0A (B-) = PD11  (PM01)
 *    PM0B (A-) = PD10  (PM02)
 *    PM0C (B+) = PD9   (PM03)
 *    PM0D (A+) = PD8   (PM04)
 *
 *  �ź���: MCU GPIO -> TLP291-4 ���� (U10) -> ULN2803A (U14) -> EXV0
 *  �߼�: GPIO HIGH = ��Ȧͨ��
 * =================================================================== */

/* --- 引脚定义 --- */
#define EXV0_PM0A_PORT   GPIOD          /* B-  PD11 */
#define EXV0_PM0A_PIN    GPIO_PIN_11
#define EXV0_PM0B_PORT   GPIOD          /* A-  PD10 */
#define EXV0_PM0B_PIN    GPIO_PIN_10
#define EXV0_PM0C_PORT   GPIOD          /* B+  PD9  */
#define EXV0_PM0C_PIN    GPIO_PIN_9
#define EXV0_PM0D_PORT   GPIOD          /* A+  PD8  */
#define EXV0_PM0D_PIN    GPIO_PIN_8

/* --- 步进参数 --- */
#define EXV_TOTAL_STEPS      500    /* VKV 全行程步数              */
#define EXV_INIT_CLOSE_STEPS 550    /* 初始化关阀步数(多50步确保全关) */
#define EXV_STEP_DELAY_MS    20     /* ÿ����ʱ (ms), Լ50 PPS     */

/* --- ���� --- */
typedef enum {
    EXV_DIR_CLOSE = 0,   /* �ط� (CW)  */
    EXV_DIR_OPEN  = 1    /* ���� (CCW) */
} EXV_Direction_t;

/* --- �ӿں��� --- */
void     BSP_EXV_Init(void);
void     BSP_EXV_DeEnergize(void);
void     BSP_EXV_Step(EXV_Direction_t dir, uint16_t steps, uint16_t delay_ms);
uint16_t BSP_EXV_GetPosition(void);
void     BSP_EXV_SetPosition(uint16_t target_steps, uint16_t delay_ms);
void     BSP_EXV_ResetToZero(void);

#endif /* BSP_EXV_H */
