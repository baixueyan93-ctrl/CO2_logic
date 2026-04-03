#ifndef BSP_INVERTER_H
#define BSP_INVERTER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *  PC12 = RS485 方向控制引脚
 *
 *  当前适配: A150 S006-3F 通用变频器 (Modbus RTU)
 *  后续切换: 老师自研变频板 (自定义协议)
 *
 *  通讯参数: 2400bps, 8N1, Modbus RTU
 *  通讯地址: 默认 0x01 (拨码 OFF/OFF)
 *
 *  A150 寄存器映射:
 *    写: 2000(0x07D0) = 转速设定 (0~120 Hz)
 *    读: 2100(0x0834) = 工作状态
 *         2101(0x0835) = 停机故障码
 *         2102(0x0836) = 报警故障码
 *         2103(0x0837) = 电机转速
 *         2108(0x083C) = 输出电流 (0~500=0~50A)
 *         2109(0x083D) = 母线电压 (0~800 Vdc)
 *         2110(0x083E) = 模块温度
 * =================================================================== */

/* --- Modbus 从机地址 (A150 拨码设定, 默认 OFF/OFF = 0x01) --- */
#define INV_MODBUS_ADDR         0x01

/* --- Modbus 功能码 --- */
#define MODBUS_FC_READ_HOLD     0x03    /* 读保持寄存器 */
#define MODBUS_FC_WRITE_SINGLE  0x06    /* 写单个寄存器 */

/* --- A150 写寄存器地址 --- */
#define INV_REG_FREQ_SET        0x07D0  /* 2000: 转速设定 (0~120 Hz) */
#define INV_REG_EC_FAN          0x07D2  /* 2002: EC风机转速 */
#define INV_REG_MOTOR_SEL       0x07D4  /* 2004: 电机选择 */
#define INV_REG_ACCEL_RATE      0x07D5  /* 2005: 升频速率 */
#define INV_REG_DECEL_RATE      0x07D6  /* 2006: 降频速率 */
#define INV_REG_ESTOP           0x07D7  /* 2007: 紧急停机 */
#define INV_REG_FREQ_UPPER      0x07D9  /* 2009: 频率上限 */
#define INV_REG_FREQ_LOWER      0x07DA  /* 2010: 频率下限 */
#define INV_REG_FREQ_MAX        0x07DB  /* 2011: 最大频率 */

/* --- A150 读寄存器地址 (Modbus地址 = 寄存器编号 - 1) --- */
#define INV_REG_STATUS          0x0833  /* 2100: 工作状态 */
#define INV_REG_FAULT_STOP      0x0834  /* 2101: 停机故障码 */
#define INV_REG_FAULT_WARN      0x0835  /* 2102: 报警故障码 */
#define INV_REG_MOTOR_SPEED     0x0836  /* 2103: 电机转速 (Hz) */
#define INV_REG_OUT_CURRENT     0x083B  /* 2108: 输出电流 (x0.1A) */
#define INV_REG_BUS_VOLTAGE     0x083C  /* 2109: 母线电压 (Vdc) */
#define INV_REG_MOD_TEMP        0x083D  /* 2110: 模块温度 */
#define INV_REG_HEATSINK_TEMP   0x083E  /* 2111: 散热器温度 */
#define INV_REG_RUN_HOURS       0x083F  /* 2112: 累计运行时间 (h) */
#define INV_REG_OUT_VOLTAGE     0x0840  /* 2113: 输出线电压 (Vrms) */
#define INV_REG_IN_VOLTAGE      0x0843  /* 2116: 输入电压 (Vrms) */
#define INV_REG_IN_CURRENT      0x0844  /* 2117: 输入电流 (x0.1A) */
#define INV_REG_OUT_POWER       0x0848  /* 2121: 输出功率 (W) */

/* --- 读寄存器批量读取: 从 2100 开始连续读取数量 --- */
#define INV_READ_START_ADDR     INV_REG_STATUS
#define INV_READ_REG_COUNT      22      /* 2100~2121, 连续22个寄存器 */

/* --- 帧长度 --- */
#define INV_TX_MAX_LEN          16      /* 发送帧最大长度 */
#define INV_RX_MAX_LEN          64      /* 接收帧最大长度 */

/* --- 频率范围 (A150: 0~120Hz) --- */
#define INV_A150_FREQ_MAX       120     /* A150 最大频率 120Hz */

/* --- 系统频率范围 (兼容旧宏, 供上层使用) --- */
#define INV_FREQ_MIN            120     /* 系统最低频率 (Hz), 映射到 A150 的 20Hz */
#define INV_FREQ_MAX            320     /* 系统最高频率 (Hz), 映射到 A150 的 120Hz */
#define INV_FRAME_LEN           16      /* 兼容旧定义 */

/* --- 通信超时 --- */
#define INV_COMM_TIMEOUT_MS     500     /* Modbus 响应超时 (ms) */

/* ===================================================================
 *  变频器状态数据结构 (从 A150 读回)
 * =================================================================== */
typedef struct {
    uint16_t status;            /* 工作状态字 (Bit0=运转, Bit1=PFC, etc.) */
    uint16_t fault_stop;        /* 停机故障码 */
    uint16_t fault_warn;        /* 报警故障码 */
    uint16_t motor_speed_hz;    /* 电机实际转速 (Hz) */
    uint16_t out_current_x10;   /* 输出电流 (x0.1A, 即500=50.0A) */
    uint16_t bus_voltage;       /* 母线电压 (Vdc) */
    int16_t  mod_temp;          /* 模块温度 (°C, 需偏移计算) */
    uint16_t out_power;         /* 输出功率 (W) */
    bool     comm_ok;           /* 通信是否成功 */
} InvStatus_t;

/* ===================================================================
 *  外部变量
 * =================================================================== */
extern UART_HandleTypeDef huart4;
extern volatile uint8_t InvAckOK;
extern InvStatus_t g_InvStatus;

/* ===================================================================
 *  公共接口
 * =================================================================== */

/* 初始化: 配置RS485方向引脚, 启动接收 */
void BSP_Inverter_Init(void);

/* 发送指令 (兼容旧接口, 内部转换为 Modbus RTU)
 *   cmd: 0x00=停机(频率写0), 0x01=启动(写初始频率), 0x02=调频
 *   freq_hz: 系统频率 (80~320), 内部映射到 A150 的 0~120Hz
 */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz);

/* 直接写 A150 频率寄存器 (0~120Hz, 写0即停机) */
void BSP_Inverter_SetFreqDirect(uint16_t freq_hz_a150);

/* 读取变频器状态 (Modbus 功能码03, 阻塞式) */
bool BSP_Inverter_ReadStatus(InvStatus_t *out);

/* Modbus CRC16 计算 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

#endif /* BSP_INVERTER_H */
