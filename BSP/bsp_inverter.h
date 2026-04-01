#ifndef BSP_INVERTER_H
#define BSP_INVERTER_H

#include "main.h"
#include <stdint.h>

/* ===================================================================
 *  变频器通信驱动 (USART1, PA9=TX, PA10=RX)
 *  16字节帧, 中断收发
 *  频率范围: 80~320 Hz (1Hz=15RPM, 1200~4800RPM)
 * =================================================================== */

#define INV_FRAME_LEN       16
#define INV_FREQ_MIN        80
#define INV_FREQ_MAX        320

extern UART_HandleTypeDef huart1;
extern uint8_t InvTxBuf[INV_FRAME_LEN];
extern uint8_t InvRxBuf[INV_FRAME_LEN];
extern volatile uint8_t InvAckOK;

void BSP_Inverter_Init(void);
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz);

#endif /* BSP_INVERTER_H */
