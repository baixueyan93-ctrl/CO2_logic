// bsp_rs485.h
// RS485 调试串口 (UART4, PC10=TX, PC11=RX, PC12=DIR)
// 原 USART1(RS485_1) 硬件故障, 已切换到 UART4(RS485_0)
#ifndef __BSP_RS485_H
#define __BSP_RS485_H
#include "main.h"

extern UART_HandleTypeDef huart4;

void BSP_RS485_Init(void);
void BSP_RS485_SendString(char *str);

#endif
