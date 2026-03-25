// bsp_rs485.c
#include "bsp_rs485.h"
#include <string.h>

extern UART_HandleTypeDef huart4;

void BSP_RS485_Init(void) {
    RS485_RX_EN(); // 默认切为接收态
}

void BSP_RS485_SendString(char *str) {
    RS485_TX_EN(); 
    // 阻塞发送，自带 TC(发送完成) 标志位等待，防止最后一个字节被截断
    HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 1000);
    RS485_RX_EN(); 
}


