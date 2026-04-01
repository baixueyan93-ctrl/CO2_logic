#ifndef BSP_INVERTER_H
#define BSP_INVERTER_H

#include "main.h"
#include <stdint.h>

/* ===================================================================
 *  变频器通信驱动 (USART1, PA9=TX, PA10=RX)
 *
 *  协议: 16字节帧, 中断收发
 *
 *  下行帧 (主控→变频板):
 *    [0]  0x55        帧头
 *    [1]  命令        0x00=关机, 0x01=启动, 0x02=调频
 *    [2]  频率低字节  freq & 0xFF
 *    [3]  频率高字节  freq >> 8
 *    [4~14] 0x00      保留
 *    [15] 0x56        帧尾
 *
 *  上行帧 (变频板→主控):
 *    变频板原样回传下行帧用于校验
 *
 *  频率范围: 80~320 Hz (1Hz=15RPM, 1200~4800RPM)
 * =================================================================== */

/* --- 命令码 --- */
#define INV_CMD_STOP        0x00
#define INV_CMD_START       0x01
#define INV_CMD_SET_FREQ    0x02

/* --- 帧常量 --- */
#define INV_FRAME_LEN       16
#define INV_FRAME_HEAD      0x55
#define INV_FRAME_TAIL      0x56

/* --- 频率限制 --- */
#define INV_FREQ_MIN        80      /* 80Hz  = 1200RPM */
#define INV_FREQ_MAX        320     /* 320Hz = 4800RPM */

/* --- 全局变量 (供中断使用) --- */
extern UART_HandleTypeDef huart1;
extern uint8_t InvTxBuf[INV_FRAME_LEN];
extern uint8_t InvRxBuf[INV_FRAME_LEN];
extern volatile uint8_t InvAckOK;
extern volatile uint8_t InvNewCmd;

/* --- 公开 API --- */
void BSP_Inverter_Init(void);
void BSP_Inverter_SendStart(uint16_t freq_hz);
void BSP_Inverter_SendStop(void);
void BSP_Inverter_SendFreq(uint16_t freq_hz);

#endif /* BSP_INVERTER_H */
