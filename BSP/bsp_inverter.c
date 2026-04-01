#include "bsp_inverter.h"
#include <string.h>

/* ===================================================================
 *  变频器通信驱动 (USART1 中断收发, 16字节帧)
 *
 *  PA9  = TXD (4851)
 *  PA10 = RXD (4851)
 *
 *  主控板(下行发送) → 变频板(上行回传)
 *  发送后变频板回传相同帧进行校验
 * =================================================================== */

/* --- USART1 句柄 --- */
UART_HandleTypeDef huart1;

/* --- 收发缓冲 --- */
uint8_t InvTxBuf[INV_FRAME_LEN];
uint8_t InvRxBuf[INV_FRAME_LEN];
volatile uint8_t InvAckOK  = 0;       /* 回传校验OK标志 */
volatile uint8_t InvNewCmd = 0;       /* 新命令标志: 1=启动 2=调频 3=停机 */

/* ===================================================================
 *  MX_USART1_UART_Init — 初始化 USART1 (PA9=TX, PA10=RX)
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    /* ---- GPIO 配置 ---- */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;    /* PA9=TX, PA10=RX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---- USART1 参数 ---- */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* ---- 使能 USART1 中断 ---- */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* ---- 开启16字节接收中断 ---- */
    HAL_UART_Receive_IT(&huart1, InvRxBuf, INV_FRAME_LEN);
}

/* ===================================================================
 *  组装16字节帧并发送
 * =================================================================== */
static void Inverter_SendFrame(uint8_t cmd, uint16_t freq_hz)
{
    uint8_t i = 0;

    /* 限幅 */
    if (freq_hz > INV_FREQ_MAX) freq_hz = INV_FREQ_MAX;
    if (cmd != INV_CMD_STOP && freq_hz < INV_FREQ_MIN) freq_hz = INV_FREQ_MIN;

    /* 清空缓冲 */
    for (i = 0; i < INV_FRAME_LEN; i++)
    {
        InvTxBuf[i] = 0x00;
    }

    /* 组帧 */
    InvTxBuf[0]  = INV_FRAME_HEAD;                    /* 0x55 帧头 */
    InvTxBuf[1]  = cmd;                                /* 命令字节 */
    InvTxBuf[2]  = (uint8_t)(freq_hz & 0xFF);         /* 频率低字节 */
    InvTxBuf[3]  = (uint8_t)((freq_hz >> 8) & 0xFF);  /* 频率高字节 */
    InvTxBuf[15] = INV_FRAME_TAIL;                     /* 0x56 帧尾 */

    /* 清除回传标志 */
    InvAckOK = 0;

    /* 发送 */
    HAL_UART_Transmit(&huart1, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);
}

/* ===================================================================
 *  公开 API — 发送启动/停机/调频命令
 * =================================================================== */
void BSP_Inverter_SendStart(uint16_t freq_hz)
{
    Inverter_SendFrame(INV_CMD_START, freq_hz);
    InvNewCmd = 1;
}

void BSP_Inverter_SendStop(void)
{
    Inverter_SendFrame(INV_CMD_STOP, 0);
    InvNewCmd = 3;
}

void BSP_Inverter_SendFreq(uint16_t freq_hz)
{
    Inverter_SendFrame(INV_CMD_SET_FREQ, freq_hz);
    InvNewCmd = 2;
}

/*********************************************************************************
 * @brief HAL_UART_RxCpltCallback, called in USART1 interrupt
 * @param huart
 * @retval None
 *********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = 0;

    if (huart->Instance == USART1)
    {
        if (InvRxBuf[0] == 0x55)
        {
            if (InvRxBuf[15] == 0x56)                            /* 首尾均正确 */
            {
                /* 逐字节校验回传帧是否与发送帧一致 */
                InvAckOK = 1;
                for (i = 0; i < INV_FRAME_LEN; i++)
                {
                    if (InvRxBuf[i] != InvTxBuf[i])
                    {
                        InvAckOK = 0;
                        break;
                    }
                }
            }
            else
            {
                InvAckOK = 0;                                     /* 帧尾错误 */
                HAL_UART_Transmit(&huart1, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);  /* 重发 */
            }
        }
        else
        {
            InvAckOK = 0;                                         /* 帧头错误 */
            HAL_UART_Transmit(&huart1, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);      /* 重发 */
        }

        /* 重新开启16字节接收中断 */
        HAL_UART_Receive_IT(&huart1, InvRxBuf, INV_FRAME_LEN);
    }
}
