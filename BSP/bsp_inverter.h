#ifndef BSP_INVERTER_H
#define BSP_INVERTER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 *  变频器通信驱动 (USART2, PA2=TX, PA3=RX)
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
 *  频率范围: 0~360 Hz, 1Hz = 15 RPM
 * =================================================================== */

/* --- 命令码 --- */
#define INV_CMD_STOP        0x00
#define INV_CMD_START       0x01
#define INV_CMD_SET_FREQ    0x02

/* --- 帧常量 --- */
#define INV_FRAME_LEN       16
#define INV_FRAME_HEAD      0x55
#define INV_FRAME_TAIL      0x56

/* --- 公开 API --- */
void     BSP_Inverter_Init(void);                   /* 初始化USART2 + 开启接收中断 */
bool     BSP_Inverter_Start(uint16_t freq_hz);      /* 启动压缩机, 设定初始频率    */
bool     BSP_Inverter_Stop(void);                   /* 停止压缩机                  */
bool     BSP_Inverter_SetFreq(uint16_t freq_hz);    /* 调整运行频率                */
bool     BSP_Inverter_IsAckOK(void);                /* 上次发送是否收到正确回传    */
void     BSP_Inverter_RxCallback(void);             /* 接收完成回调 (中断中调用)   */

#endif /* BSP_INVERTER_H */
