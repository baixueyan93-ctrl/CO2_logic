#include "bsp_inverter.h"

/* ===================================================================
 *  变频器通信驱动 (USART1, PA9=TX, PA10=RX, 16字节帧)
 * =================================================================== */

UART_HandleTypeDef huart1;

uint8_t InvTxBuf[INV_FRAME_LEN];
uint8_t InvRxBuf[INV_FRAME_LEN];
volatile uint8_t InvAckOK = 0;        /* 回传校验OK */

/* ===================================================================
 *  初始化 USART1
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);   /* FreeRTOS下最高可用优先级 */
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    HAL_UART_Receive_IT(&huart1, InvRxBuf, INV_FRAME_LEN);
}

/* ===================================================================
 *  组帧并发送16字节
 *  cmd: 0x00=关机, 0x01=启动, 0x02=调频
 *  freq_hz: 频率 (80~320)
 * =================================================================== */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz)
{
    uint8_t i = 0;

    if (freq_hz > INV_FREQ_MAX) freq_hz = INV_FREQ_MAX;
    if (cmd != 0x00 && freq_hz < INV_FREQ_MIN) freq_hz = INV_FREQ_MIN;

    for (i = 0; i < INV_FRAME_LEN; i++) { InvTxBuf[i] = 0x00; }

    InvTxBuf[0]  = 0x55;
    InvTxBuf[1]  = cmd;
    InvTxBuf[2]  = (uint8_t)(freq_hz & 0xFF);
    InvTxBuf[3]  = (uint8_t)((freq_hz >> 8) & 0xFF);
    InvTxBuf[15] = 0x56;

    InvAckOK = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);
}

/*********************************************************************************
 * @brief HAL_UART_RxCpltCallback, called in USART1 interrupt
 * @param None
 * @retval None
 *********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = 0;

    if (huart->Instance == USART1)
    {
        if (InvRxBuf[0] == 0x55)
        {
            if (InvRxBuf[15] == 0x56)
            {
                InvAckOK = 1;
                for (i = 0; i < INV_FRAME_LEN; i++)
                {
                    if (InvRxBuf[i] != InvTxBuf[i])
                    {
                        InvAckOK = 0;                              /* 校验不一致 */
                        break;
                    }
                }
            }
            else
            {
                InvAckOK = 0;
                HAL_UART_Transmit(&huart1, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);   /* 帧尾错,重发 */
            }
        }
        else
        {
            InvAckOK = 0;
            HAL_UART_Transmit(&huart1, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);       /* 帧头错,重发 */
        }

        HAL_UART_Receive_IT(&huart1, InvRxBuf, INV_FRAME_LEN);
    }
    else if (huart->Instance == UART4)
    {
        /* RS485/Modbus 接收回调 (原 task_rs485_log.c) */
        extern uint8_t rx_buffer[128];
        extern uint16_t rx_index;
        extern volatile uint8_t rx_complete;
        extern uint8_t rx_byte;
        extern UART_HandleTypeDef huart4;

        rx_buffer[rx_index++] = rx_byte;

        if (rx_byte == '\n' || rx_byte == '\r' || rx_index >= 127) {
            rx_buffer[rx_index] = '\0';
            rx_complete = 1;
        } else {
            HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
        }
    }
}
