// bsp_rs485.h
#ifndef __BSP_RS485_H
#define __BSP_RS485_H
#include "main.h"

// PC12 ∑ΩœÚ«–ªª∫Í
#define RS485_TX_EN()  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET)
#define RS485_RX_EN()  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET)

void BSP_RS485_Init(void);
void BSP_RS485_SendString(char *str);

#endif


