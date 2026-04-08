#ifndef BSP_INVERTER_H
#define BSP_INVERTER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *  PC12 = RS485 方向控制引脚
 *
 *  适配: 老师自研变频板 (自定义16字节协议)
 *
 *  通讯参数: 9600bps, 8N1, RS485半双工
 *
 *  下行帧格式 (主控→变频板, 16字节):
 *    [0]  0x55        帧头
 *    [1]  CMD         命令: 0x00=停机, 0x01=启动, 0x02=调频
 *    [2]  FREQ_LO     频率低字节 (例: 258Hz → 0x02)
 *    [3]  FREQ_HI     频率高字节 (例: 258Hz → 0x01)
 *    [4~14] 0x00      保留 (填0)
 *    [15] 0x56        帧尾
 *
 *  上行帧格式 (变频板→主控, 16字节):
 *    原样回传下行帧内容, 用于校验传输正确性
 *
 *  频率范围: 0~360Hz, 1Hz = 15转 (8极电机)
 *  压缩机工作范围: 120~320Hz (1800~4800转)
 * =================================================================== */

/* --- 帧格式常量 --- */
#define INV_FRAME_LEN           16      /* 帧总长度 (字节) */
#define INV_FRAME_HEAD          0x55    /* 帧头 */
#define INV_FRAME_TAIL          0x56    /* 帧尾 */

/* --- 命令字节定义 --- */
#define INV_CMD_STOP            0x00    /* 停机 */
#define INV_CMD_START           0x01    /* 启动 */
#define INV_CMD_SET_FREQ        0x02    /* 调频 */

/* --- 帧内字节偏移 --- */
#define INV_OFS_HEAD            0       /* 帧头 */
#define INV_OFS_CMD             1       /* 命令 */
#define INV_OFS_FREQ_LO         2       /* 频率低字节 */
#define INV_OFS_FREQ_HI         3       /* 频率高字节 */
#define INV_OFS_TAIL            15      /* 帧尾 */

/* --- 频率范围 --- */
#define INV_FREQ_ABS_MAX        360     /* 变频器最大频率 (Hz) */
#define INV_FREQ_MIN            120     /* 系统最低工作频率 (Hz) */
#define INV_FREQ_MAX            320     /* 系统最高工作频率 (Hz) */

/* --- 通信超时 --- */
#define INV_COMM_TIMEOUT_MS     200     /* 回传超时 (ms) */

/* --- 发送/接收缓冲区 --- */
#define INV_TX_MAX_LEN          16
#define INV_RX_MAX_LEN          16

/* ===================================================================
 *  变频器状态数据结构
 *  (老师板子暂无状态回读, 仅记录通信状态和最后发送的频率)
 * =================================================================== */
typedef struct {
    uint16_t last_freq_hz;      /* 最后一次发送的频率 (Hz) */
    uint8_t  last_cmd;          /* 最后一次发送的命令 */
    bool     echo_ok;           /* 上次发送的echo校验是否通过 */
    bool     comm_ok;           /* 通信是否正常 (echo_ok的别名, 兼容上层) */
    uint8_t  fail_reason;       /* 失败原因: 0=无, 1=超时无回应, 2=回传不一致 */
    uint16_t motor_speed_hz;    /* 兼容字段, 无实际回读, 暂设为0 */
    uint16_t fault_stop;        /* 兼容字段, 暂设为0 */
    uint16_t fault_warn;        /* 兼容字段, 暂设为0 */
    uint16_t out_current_x10;   /* 兼容字段, 暂设为0 */
    uint16_t bus_voltage;       /* 兼容字段, 暂设为0 */
} InvStatus_t;

/* ===================================================================
 *  外部变量
 * =================================================================== */
extern UART_HandleTypeDef huart4;
extern volatile uint8_t InvAckOK;
extern InvStatus_t g_InvStatus;

/* ===================================================================
 *  公共接口 (保持与上层代码兼容)
 * =================================================================== */

/* 初始化: 配置RS485方向引脚 */
void BSP_Inverter_Init(void);

/* 发送指令
 *   cmd: 0x00=停机, 0x01=启动, 0x02=调频
 *   freq_hz: 目标频率 (Hz), 启动时写120, 调频时写目标值
 */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz);

/* 读取变频器状态
 * 老师板子暂无主动状态回读, 此函数返回最近一次echo结果
 */
bool BSP_Inverter_ReadStatus(InvStatus_t *out);

#endif /* BSP_INVERTER_H */
