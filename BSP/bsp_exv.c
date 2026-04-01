#include "bsp_exv.h"
#include "FreeRTOS.h"
#include "task.h"

/* ===================================================================
 *  �ع� VKV �������ͷ� �����������
 *
 *  ������4�ಽ�����, 2������ȫ��������
 *  ÿ��ͬʱ��ͨ��������, ��������Ť��
 *
 *  ȫ�������� (2�ർͨ):
 *    Step 0: A+(PM0D)=1, B+(PM0C)=1, A-(PM0B)=0, B-(PM0A)=0
 *    Step 1: A-(PM0B)=1, B+(PM0C)=1, A+(PM0D)=0, B-(PM0A)=0
 *    Step 2: A-(PM0B)=1, B-(PM0A)=1, A+(PM0D)=0, B+(PM0C)=0
 *    Step 3: A+(PM0D)=1, B-(PM0A)=1, A-(PM0B)=0, B+(PM0C)=0
 * =================================================================== */

/* ��ǰλ�� (0 = ȫ��, EXV_TOTAL_STEPS = ȫ��) */
static volatile uint16_t s_exv_position = 0;

/* ��ǰ�������� (0~3) */
static volatile uint8_t s_phase_index = 0;

/* �����: ÿ��Ԫ�� = {PM0A(B-), PM0B(A-), PM0C(B+), PM0D(A+)} */
static const uint8_t PHASE_TABLE[4][4] = {
    /* PM0A  PM0B  PM0C  PM0D */
    {  0,    0,    1,    1  },  /* Step 0: A+ & B+ */
    {  0,    1,    1,    0  },  /* Step 1: A- & B+ */
    {  1,    1,    0,    0  },  /* Step 2: A- & B- */
    {  1,    0,    0,    1  },  /* Step 3: A+ & B- */
};

/* ------------------------------------------------------------------ */
static void exv_set_phase(uint8_t phase_idx)
{
    const uint8_t *ph = PHASE_TABLE[phase_idx & 0x03];

    HAL_GPIO_WritePin(EXV0_PM0A_PORT, EXV0_PM0A_PIN,
                      ph[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXV0_PM0B_PORT, EXV0_PM0B_PIN,
                      ph[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXV0_PM0C_PORT, EXV0_PM0C_PIN,
                      ph[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXV0_PM0D_PORT, EXV0_PM0D_PIN,
                      ph[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===================================================================
 *  BSP_EXV_Init - ��ʼ�� GPIO ���� (�������, Ĭ�ϵ͵�ƽ)
 * =================================================================== */
void BSP_EXV_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 所有引脚先拉低 */
    HAL_GPIO_WritePin(GPIOD,
        EXV0_PM0A_PIN | EXV0_PM0B_PIN | EXV0_PM0C_PIN | EXV0_PM0D_PIN,
        GPIO_PIN_RESET);

    /* 配置 PD8, PD9, PD10, PD11 为推挽输出 */
    GPIO_InitStruct.Pin   = EXV0_PM0A_PIN | EXV0_PM0B_PIN |
                            EXV0_PM0C_PIN | EXV0_PM0D_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    s_exv_position = 0;
    s_phase_index  = 0;
}

/* ===================================================================
 *  BSP_EXV_DeEnergize - �ϵ�������Ȧ (ʡ��, ���ٷ���)
 * =================================================================== */
void BSP_EXV_DeEnergize(void)
{
    HAL_GPIO_WritePin(GPIOD,
        EXV0_PM0A_PIN | EXV0_PM0B_PIN | EXV0_PM0C_PIN | EXV0_PM0D_PIN,
        GPIO_PIN_RESET);
}

/* ===================================================================
 *  BSP_EXV_Step - ִ��ָ������
 *  @param dir      : EXV_DIR_OPEN / EXV_DIR_CLOSE
 *  @param steps    : ����
 *  @param delay_ms : ÿ����ʱ (ms), ���� >= 15
 * =================================================================== */
void BSP_EXV_Step(EXV_Direction_t dir, uint16_t steps, uint16_t delay_ms)
{
    for (uint16_t i = 0; i < steps; i++) {
        if (dir == EXV_DIR_OPEN) {
            s_phase_index = (s_phase_index + 1) & 0x03;
            if (s_exv_position < EXV_TOTAL_STEPS)
                s_exv_position++;
        } else {
            s_phase_index = (s_phase_index + 3) & 0x03; /* -1 mod 4 */
            if (s_exv_position > 0)
                s_exv_position--;
        }

        exv_set_phase(s_phase_index);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/* ===================================================================
 *  BSP_EXV_GetPosition - ��ȡ��ǰλ�� (����)
 * =================================================================== */
uint16_t BSP_EXV_GetPosition(void)
{
    return s_exv_position;
}

/* ===================================================================
 *  BSP_EXV_SetPosition - �ƶ���Ŀ��λ��
 * =================================================================== */
void BSP_EXV_SetPosition(uint16_t target_steps, uint16_t delay_ms)
{
    if (target_steps > EXV_TOTAL_STEPS)
        target_steps = EXV_TOTAL_STEPS;

    if (target_steps > s_exv_position) {
        BSP_EXV_Step(EXV_DIR_OPEN,
                     target_steps - s_exv_position, delay_ms);
    } else if (target_steps < s_exv_position) {
        BSP_EXV_Step(EXV_DIR_CLOSE,
                     s_exv_position - target_steps, delay_ms);
    }
}

/* ===================================================================
 *  BSP_EXV_ResetToZero - ����: �ط� 550 �� (ȷ����еȫ��)
 * =================================================================== */
void BSP_EXV_ResetToZero(void)
{
    /* ����� s_exv_position, ֱ�Ӷಽ�ط�ȷ������ */
    for (uint16_t i = 0; i < EXV_INIT_CLOSE_STEPS; i++) {
        s_phase_index = (s_phase_index + 3) & 0x03;
        exv_set_phase(s_phase_index);
        vTaskDelay(pdMS_TO_TICKS(EXV_STEP_DELAY_MS));
    }
    s_exv_position = 0;
    BSP_EXV_DeEnergize();
}
