// bsp_rs485.c
// RS485 调试串口 — 已从 USART1(RS485_1) 切换到 UART4(RS485_0)
// UART4: PC10=TX, PC11=RX, PC12=RS485方向控制
// 波特率 9600, 8N1
#include "bsp_rs485.h"
#include <string.h>

/* RS485 方向控制 (PC12): HIGH=发送, LOW=接收 */
#define RS485_DIR_PORT  GPIOC
#define RS485_DIR_PIN   GPIO_PIN_12

static void RS485_DebugSetTx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET);
}

static void RS485_DebugSetRx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
}

void BSP_RS485_Init(void)
{
    /* UART4 已在 MX_UART4_Init() 中初始化 (9600, 8N1)
     * PC12 方向引脚已在 MX_GPIO_Init() 中配置
     * 这里只需确保默认为接收模式, 并启用中断 */
    RS485_DebugSetRx();

    /* 启用 UART4 中断 (用于调试串口逐字节接收) */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

void BSP_RS485_SendString(char *str)
{
    RS485_DebugSetTx();
    HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 1000);
    RS485_DebugSetRx();
}
