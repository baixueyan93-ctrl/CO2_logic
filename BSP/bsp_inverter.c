#include "bsp_inverter.h"

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *  UART4 已由 CubeMX 在 usart.c 中初始化 (9600, 8N1)
 *  PC12 = RS485 方向控制引脚
 * =================================================================== */

uint8_t InvTxBuf[INV_FRAME_LEN];
uint8_t InvRxBuf[INV_FRAME_LEN];
volatile uint8_t InvAckOK = 0;        /* 回传校验OK */

/* ===================================================================
 *  初始化: 启动UART4中断接收 (UART4硬件已由CubeMX初始化)
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    /* RS485 默认为接收模式 */
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);

    /* 启动中断接收, 等待回传16字节 */
    HAL_UART_Receive_IT(&huart4, InvRxBuf, INV_FRAME_LEN);
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

    /* RS485 切换为发送模式 */
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);

    HAL_UART_Transmit(&huart4, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);

    /* 发送完毕, 切回接收模式 */
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

/*********************************************************************************
 * @brief HAL_UART_RxCpltCallback (统一回调)
 *        UART4  = 变频器 16字节帧校验
 *        USART1 = RS485调试 逐字节接收
 *********************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = 0;

    if (huart->Instance == UART4)
    {
        if (InvRxBuf[0] == 0x55)
        {
            if (InvRxBuf[15] == 0x56)
            {
                InvAckOK = 1;                          /* 首尾正确, 通信成功 */
            }
            else
            {
                InvAckOK = 0;
                HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit(&huart4, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);   /* 帧尾错, 重发原始帧 */
                HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
            }
        }
        else
        {
            InvAckOK = 0;
            HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
            HAL_UART_Transmit(&huart4, (uint8_t*)InvTxBuf, INV_FRAME_LEN, 1000);       /* 帧头错, 重发原始帧 */
            HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
        }

        HAL_UART_Receive_IT(&huart4, InvRxBuf, INV_FRAME_LEN);
    }
    else if (huart->Instance == USART1)
    {
        /* RS485 调试串口: 逐字节接收 */
        extern uint8_t rx_buffer[128];
        extern uint16_t rx_index;
        extern volatile uint8_t rx_complete;
        extern uint8_t rx_byte;

        rx_buffer[rx_index++] = rx_byte;

        if (rx_byte == '\n' || rx_byte == '\r' || rx_index >= 127) {
            rx_buffer[rx_index] = '\0';
            rx_complete = 1;
        } else {
            extern UART_HandleTypeDef huart1;
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
    }
}
