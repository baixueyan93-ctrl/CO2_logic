#include "bsp_inverter.h"
#include <string.h>

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *
 *  当前适配: A150 S006-3F 通用变频器 (Modbus RTU, 2400bps)
 *  后续切换: 老师自研变频板 (自定义协议) — 只需修改本文件
 *
 *  Modbus RTU 帧格式:
 *    [地址1B][功能码1B][数据nB][CRC16-2B(低位在前)]
 *
 *  功能码03 (读保持寄存器):
 *    请求: [Addr][03][RegAddrH][RegAddrL][RegCountH][RegCountL][CRCL][CRCH]
 *    响应: [Addr][03][ByteCount][Data...][CRCL][CRCH]
 *
 *  功能码06 (写单个寄存器):
 *    请求: [Addr][06][RegAddrH][RegAddrL][ValueH][ValueL][CRCL][CRCH]
 *    响应: 原样回传 (echo)
 *
 *  通讯示例 (来自A150规格书):
 *    写70Hz: 01 06 07 CF 00 46 39 73
 *    写0Hz:  01 06 07 CF 00 00 B8 81
 *    读母线: 01 03 08 3C 00 01 46 66
 *
 *  注意: A150 规格书中写频率用的寄存器地址是 0x07CF,
 *        但寄存器表标注是 2000 = 0x07D0。以通讯示例为准, 使用 0x07CF。
 * =================================================================== */

/* ===================================================================
 *  全局变量
 * =================================================================== */
volatile uint8_t InvAckOK = 0;
InvStatus_t g_InvStatus = {0};

/* ===================================================================
 *  内部变量
 * =================================================================== */
static uint8_t s_tx_buf[INV_TX_MAX_LEN];
static uint8_t s_rx_buf[INV_RX_MAX_LEN];

/* A150 写频率寄存器地址 — 以规格书通讯示例为准 (0x07CF) */
#define INV_REG_FREQ_SET_ACTUAL  0x07CF

/* ===================================================================
 *  Modbus CRC16 (多项式 0xA001, 标准 Modbus RTU)
 * =================================================================== */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;  /* 低字节在前 */
}

/* ===================================================================
 *  RS485 方向控制
 * =================================================================== */
static void RS485_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
}

static void RS485_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

/* ===================================================================
 *  Modbus RTU 写单个寄存器 (功能码 0x06)
 *
 *  发送: [Addr][06][RegH][RegL][ValH][ValL][CRCL][CRCH]  共8字节
 *  接收: 原样回传 8字节
 *
 *  返回: true=收到正确回传, false=超时或CRC错误
 * =================================================================== */
static bool Modbus_WriteSingleReg(uint16_t reg_addr, uint16_t value)
{
    uint16_t crc;

    /* 组帧 */
    s_tx_buf[0] = INV_MODBUS_ADDR;
    s_tx_buf[1] = MODBUS_FC_WRITE_SINGLE;
    s_tx_buf[2] = (uint8_t)(reg_addr >> 8);
    s_tx_buf[3] = (uint8_t)(reg_addr & 0xFF);
    s_tx_buf[4] = (uint8_t)(value >> 8);
    s_tx_buf[5] = (uint8_t)(value & 0xFF);

    crc = Modbus_CRC16(s_tx_buf, 6);
    s_tx_buf[6] = (uint8_t)(crc & 0xFF);        /* CRC 低字节在前 */
    s_tx_buf[7] = (uint8_t)((crc >> 8) & 0xFF);

    /* RS485 切换为发送模式 */
    RS485_SetTx();

    /* 发送 8 字节 */
    HAL_UART_Transmit(&huart4, s_tx_buf, 8, INV_COMM_TIMEOUT_MS);

    /* 发送完毕, 切回接收模式 */
    RS485_SetRx();

    /* 等待回传 (A150 回传原样 8 字节) */
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart4, s_rx_buf, 8,
                                                 INV_COMM_TIMEOUT_MS);

    if (status != HAL_OK) {
        InvAckOK = 0;
        return false;
    }

    /* 校验 CRC */
    crc = Modbus_CRC16(s_rx_buf, 6);
    if (s_rx_buf[6] != (uint8_t)(crc & 0xFF) ||
        s_rx_buf[7] != (uint8_t)((crc >> 8) & 0xFF)) {
        InvAckOK = 0;
        return false;
    }

    /* 校验回传内容 (应与发送一致) */
    if (memcmp(s_tx_buf, s_rx_buf, 6) != 0) {
        InvAckOK = 0;
        return false;
    }

    InvAckOK = 1;
    return true;
}

/* ===================================================================
 *  Modbus RTU 读保持寄存器 (功能码 0x03)
 *
 *  发送: [Addr][03][RegH][RegL][CountH][CountL][CRCL][CRCH]  共8字节
 *  接收: [Addr][03][ByteCount][Data...][CRCL][CRCH]
 *        ByteCount = Count * 2
 *        总接收长度 = 3 + ByteCount + 2 = 5 + Count*2
 *
 *  reg_addr: 起始寄存器地址
 *  count:    要读取的寄存器数量
 *  out_data: 输出缓冲区 (至少 count 个 uint16_t)
 *
 *  返回: true=读取成功, false=超时或CRC错误
 * =================================================================== */
static bool Modbus_ReadHoldRegs(uint16_t reg_addr, uint16_t count,
                                 uint16_t *out_data)
{
    uint16_t crc;
    uint16_t rx_len = 5 + count * 2;  /* 预期接收总长度 */

    if (rx_len > INV_RX_MAX_LEN) {
        return false;
    }

    /* 组帧 */
    s_tx_buf[0] = INV_MODBUS_ADDR;
    s_tx_buf[1] = MODBUS_FC_READ_HOLD;
    s_tx_buf[2] = (uint8_t)(reg_addr >> 8);
    s_tx_buf[3] = (uint8_t)(reg_addr & 0xFF);
    s_tx_buf[4] = (uint8_t)(count >> 8);
    s_tx_buf[5] = (uint8_t)(count & 0xFF);

    crc = Modbus_CRC16(s_tx_buf, 6);
    s_tx_buf[6] = (uint8_t)(crc & 0xFF);
    s_tx_buf[7] = (uint8_t)((crc >> 8) & 0xFF);

    /* RS485 切换为发送模式 */
    RS485_SetTx();

    /* 发送 8 字节 */
    HAL_UART_Transmit(&huart4, s_tx_buf, 8, INV_COMM_TIMEOUT_MS);

    /* 发送完毕, 切回接收模式 */
    RS485_SetRx();

    /* 等待回传 */
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart4, s_rx_buf, rx_len,
                                                 INV_COMM_TIMEOUT_MS);

    if (status != HAL_OK) {
        return false;
    }

    /* 校验: 地址 + 功能码 + 字节数 */
    if (s_rx_buf[0] != INV_MODBUS_ADDR ||
        s_rx_buf[1] != MODBUS_FC_READ_HOLD ||
        s_rx_buf[2] != (uint8_t)(count * 2)) {
        return false;
    }

    /* 校验 CRC */
    crc = Modbus_CRC16(s_rx_buf, rx_len - 2);
    if (s_rx_buf[rx_len - 2] != (uint8_t)(crc & 0xFF) ||
        s_rx_buf[rx_len - 1] != (uint8_t)((crc >> 8) & 0xFF)) {
        return false;
    }

    /* 解析数据 (高字节在前) */
    for (uint16_t i = 0; i < count; i++) {
        out_data[i] = ((uint16_t)s_rx_buf[3 + i * 2] << 8) |
                       (uint16_t)s_rx_buf[4 + i * 2];
    }

    return true;
}

/* ===================================================================
 *  初始化
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    /* RS485 默认为接收模式 */
    RS485_SetRx();

    /* Modbus使用阻塞式收发, 禁用UART4中断防止Overrun错误卡死系统 */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_ERR);

    /* 清空状态 */
    InvAckOK = 0;
    memset(&g_InvStatus, 0, sizeof(g_InvStatus));
}

/* ===================================================================
 *  BSP_Inverter_Send — 兼容旧接口
 *
 *  上层逻辑调用方式不变:
 *    cmd=0x00, freq=0   → 停机 (写频率0)
 *    cmd=0x01, freq=120 → 启动 (写初始频率)
 *    cmd=0x02, freq=xxx → 调频 (写目标频率)
 *
 *  内部将系统频率 (120~320 Hz) 映射到 A150 频率 (0~120 Hz):
 *
 *    系统频率范围: SET_FREQ_MIN(120) ~ SET_FREQ_MAX(320)
 *    A150频率范围: 0 ~ 120 Hz
 *
 *    映射公式: a150_freq = (sys_freq - 120) * 120 / (320 - 120)
 *              a150_freq = (sys_freq - 120) * 120 / 200
 *              a150_freq = (sys_freq - 120) * 3 / 5
 *
 *    120Hz → 0Hz   (最低)
 *    220Hz → 60Hz  (中间)
 *    320Hz → 120Hz (最高)
 *
 *  注意: cmd=0x00 时直接写0, 不做映射
 * =================================================================== */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz)
{
    uint16_t a150_freq = 0;

    if (cmd == 0x00) {
        /* 停机: 写频率 0 */
        a150_freq = 0;
    } else {
        /* 直接透传: 系统频率 = A150频率, 不做映射 */
        a150_freq = freq_hz;
    }

    /* 上限保护 */
    if (a150_freq > INV_A150_FREQ_MAX) {
        a150_freq = INV_A150_FREQ_MAX;
    }

    /* 通过 Modbus RTU 写频率寄存器 */
    BSP_Inverter_SetFreqDirect(a150_freq);
}

/* ===================================================================
 *  直接写 A150 频率寄存器 (0~120Hz)
 *  写0即停机, 写 > 频率下限即启动
 * =================================================================== */
void BSP_Inverter_SetFreqDirect(uint16_t freq_hz_a150)
{
    if (freq_hz_a150 > INV_A150_FREQ_MAX) {
        freq_hz_a150 = INV_A150_FREQ_MAX;
    }

    Modbus_WriteSingleReg(INV_REG_FREQ_SET_ACTUAL, freq_hz_a150);
}

/* ===================================================================
 *  读取变频器状态 (批量读关键寄存器)
 *
 *  读取 2100~2113 共14个寄存器, 覆盖:
 *    状态、故障码、转速、电流、电压、温度、功率等
 *
 *  返回: true=读取成功
 * =================================================================== */
bool BSP_Inverter_ReadStatus(InvStatus_t *out)
{
    uint16_t regs[14];  /* 2100~2113 */

    if (!Modbus_ReadHoldRegs(INV_READ_START_ADDR, 14, regs)) {
        if (out) {
            out->comm_ok = false;
        }
        return false;
    }

    if (out) {
        out->status          = regs[0];   /* 2100: 工作状态 */
        out->fault_stop      = regs[1];   /* 2101: 停机故障码 */
        out->fault_warn      = regs[2];   /* 2102: 报警故障码 */
        out->motor_speed_hz  = regs[3];   /* 2103: 电机转速 */
        out->out_current_x10 = regs[8];   /* 2108: 输出电流 */
        out->bus_voltage     = regs[9];   /* 2109: 母线电压 */
        out->mod_temp        = (int16_t)(regs[10] - 55); /* 2110: 模块温度, 偏移-55 */
        out->out_power       = 0;         /* 2121 不在本次批量读取范围内 */
        out->comm_ok         = true;
    }

    /* 同步到全局状态 */
    g_InvStatus = *out;
    InvAckOK = 1;

    return true;
}

/* ===================================================================
 *  UART 接收完成回调 (统一回调, 处理多个 UART 实例)
 *
 *  UART4  = 变频器 Modbus — 当前使用阻塞式收发, 此回调仅作备用
 *  USART1 = RS485 调试串口 — 逐字节中断接收
 * =================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        /* Modbus 当前使用阻塞式收发, 暂不需要中断回调处理 */
        /* 后续如果改为 DMA/中断方式, 在此处理响应解析 */
    }
    else if (huart->Instance == USART1)
    {
        /* RS485 调试串口: 逐字节接收 */
        extern uint8_t rx_buffer[128];
        extern uint16_t rx_index;
        extern volatile uint8_t rx_complete;
        extern uint8_t rx_byte;

        rx_buffer[rx_index++] = rx_byte;

        if (rx_byte == '\n' || rx_byte == '\r' || rx_index >= 127) {
            rx_buffer[rx_index] = '\0';
            rx_complete = 1;
        } else {
            extern UART_HandleTypeDef huart1;
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
    }
}
