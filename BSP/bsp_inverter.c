#include "bsp_inverter.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ===================================================================
 *  变频器通信驱动 (USART2 中断收发, 16字节帧)
 *
 *  主控板(下行发送) → 变频板(上行回传)
 *  发送后等待变频板回传相同帧进行校验
 * =================================================================== */

/* --- USART2 句柄 --- */
UART_HandleTypeDef huart2;

/* --- 收发缓冲 --- */
static uint8_t s_tx_buf[INV_FRAME_LEN];
static uint8_t s_rx_buf[INV_FRAME_LEN];
static volatile bool s_ack_ok   = false;   /* 回传校验结果   */
static volatile bool s_rx_done  = false;   /* 接收完成标志   */

/* ===================================================================
 *  BSP_Inverter_Init — 初始化 USART2 (PA2=TX, PA3=RX) + 开启接收中断
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    /* ---- GPIO 配置 ---- */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;   /* PA2=TX, PA3=RX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---- USART2 参数 ---- */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 9600;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* ---- 使能 USART2 中断 ---- */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* ---- 开启16字节接收中断 ---- */
    HAL_UART_Receive_IT(&huart2, s_rx_buf, INV_FRAME_LEN);
}

/* ===================================================================
 *  组装帧并发送
 * =================================================================== */
static bool Inverter_SendFrame(uint8_t cmd, uint16_t freq_hz)
{
    /* 组帧 */
    memset(s_tx_buf, 0x00, INV_FRAME_LEN);
    s_tx_buf[0]  = INV_FRAME_HEAD;       /* 0x55 */
    s_tx_buf[1]  = cmd;                   /* 命令 */
    s_tx_buf[2]  = (uint8_t)(freq_hz & 0xFF);        /* 频率低字节 */
    s_tx_buf[3]  = (uint8_t)((freq_hz >> 8) & 0xFF); /* 频率高字节 */
    s_tx_buf[15] = INV_FRAME_TAIL;       /* 0x56 */

    /* 清除上次状态 */
    s_ack_ok  = false;
    s_rx_done = false;

    /* 发送 */
    if (HAL_UART_Transmit(&huart2, s_tx_buf, INV_FRAME_LEN, 100) != HAL_OK) {
        return false;
    }

    /* 等待变频板回传 (最多200ms) */
    for (uint8_t i = 0; i < 40; i++) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (s_rx_done) {
            return s_ack_ok;
        }
    }
    return false;  /* 超时 */
}

/* ===================================================================
 *  公开 API
 * =================================================================== */
bool BSP_Inverter_Start(uint16_t freq_hz)
{
    return Inverter_SendFrame(INV_CMD_START, freq_hz);
}

bool BSP_Inverter_Stop(void)
{
    return Inverter_SendFrame(INV_CMD_STOP, 0);
}

bool BSP_Inverter_SetFreq(uint16_t freq_hz)
{
    return Inverter_SendFrame(INV_CMD_SET_FREQ, freq_hz);
}

bool BSP_Inverter_IsAckOK(void)
{
    return s_ack_ok;
}

/* ===================================================================
 *  BSP_Inverter_RxCallback — 接收完成回调
 *
 *  在 HAL_UART_RxCpltCallback 中调用
 *  校验: 回传帧必须与发送帧完全一致
 * =================================================================== */
void BSP_Inverter_RxCallback(void)
{
    /* 校验帧头帧尾 */
    if (s_rx_buf[0] == INV_FRAME_HEAD && s_rx_buf[15] == INV_FRAME_TAIL) {
        /* 逐字节比较发送帧和回传帧 */
        s_ack_ok = (memcmp(s_tx_buf, s_rx_buf, INV_FRAME_LEN) == 0);
    } else {
        s_ack_ok = false;
    }

    s_rx_done = true;

    /* 重新开启接收, 等待下一次回传 */
    HAL_UART_Receive_IT(&huart2, s_rx_buf, INV_FRAME_LEN);
}
