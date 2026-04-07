#include "bsp_inverter.h"
#include <string.h>

/* ===================================================================
 *  变频器通信驱动 (UART4, PC10=TX, PC11=RX, RS485)
 *
 *  适配: 老师自研变频板 (自定义16字节协议, 9600bps)
 *
 *  下行帧 (主控→变频板, 16字节):
 *    [0]  0x55        帧头
 *    [1]  CMD         0x00=停机, 0x01=启动, 0x02=调频
 *    [2]  FREQ_LO     频率低字节
 *    [3]  FREQ_HI     频率高字节
 *    [4~14] 0x00      保留
 *    [15] 0x56        帧尾
 *
 *  上行帧 (变频板→主控, 16字节):
 *    原样回传, 主控校验是否一致
 *
 *  频率编码: 小端序 uint16
 *    例: 258Hz → [2]=0x02(低), [3]=0x01(高)  (256+2=258)
 *    例: 120Hz → [2]=0x78(低), [3]=0x00(高)
 *    例: 320Hz → [2]=0x40(低), [3]=0x01(高)
 * =================================================================== */

/* ===================================================================
 *  全局变量
 * =================================================================== */
volatile uint8_t InvAckOK = 0;
InvStatus_t g_InvStatus = {0};

/* ===================================================================
 *  内部变量
 * =================================================================== */
static uint8_t s_tx_buf[INV_FRAME_LEN];
static uint8_t s_rx_buf[INV_FRAME_LEN];

/* ===================================================================
 *  RS485 方向控制 (PC12)
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
 *  组装16字节下行帧
 *
 *  cmd:     命令字节 (0x00/0x01/0x02)
 *  freq_hz: 频率值 (Hz), 小端序拆分到 byte[2](低) byte[3](高)
 * =================================================================== */
static void BuildFrame(uint8_t cmd, uint16_t freq_hz)
{
    memset(s_tx_buf, 0x00, INV_FRAME_LEN);

    s_tx_buf[INV_OFS_HEAD]    = INV_FRAME_HEAD;       /* 0x55 */
    s_tx_buf[INV_OFS_CMD]     = cmd;
    s_tx_buf[INV_OFS_FREQ_LO] = (uint8_t)(freq_hz & 0xFF);         /* 低字节 */
    s_tx_buf[INV_OFS_FREQ_HI] = (uint8_t)((freq_hz >> 8) & 0xFF);  /* 高字节 */
    s_tx_buf[INV_OFS_TAIL]    = INV_FRAME_TAIL;       /* 0x56 */
}

/* ===================================================================
 *  发送帧并等待echo回传, 校验一致性
 *
 *  返回: true = echo与发送一致, false = 超时或内容不一致
 * =================================================================== */
static bool SendAndVerifyEcho(void)
{
    /* RS485 切发送模式 */
    RS485_SetTx();

    /* 发送16字节 */
    HAL_UART_Transmit(&huart4, s_tx_buf, INV_FRAME_LEN, INV_COMM_TIMEOUT_MS);

    /* 等待移位寄存器发完最后一个字节再切RX
     * 9600bps下1字节≈1.04ms, 等2ms确保停止位发完 */
    while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET) {}

    /* 切回接收模式 */
    RS485_SetRx();

    /* 等待变频板回传16字节 */
    memset(s_rx_buf, 0, INV_FRAME_LEN);
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart4, s_rx_buf, INV_FRAME_LEN,
                                                 INV_COMM_TIMEOUT_MS);

    if (status != HAL_OK) {
        InvAckOK = 0;
        g_InvStatus.echo_ok = false;
        g_InvStatus.comm_ok = false;
        return false;
    }

    /* 校验: echo应与发送内容完全一致 */
    if (memcmp(s_tx_buf, s_rx_buf, INV_FRAME_LEN) != 0) {
        InvAckOK = 0;
        g_InvStatus.echo_ok = false;
        g_InvStatus.comm_ok = false;
        return false;
    }

    /* echo校验通过 */
    InvAckOK = 1;
    g_InvStatus.echo_ok = true;
    g_InvStatus.comm_ok = true;
    return true;
}

/* ===================================================================
 *  初始化
 * =================================================================== */
void BSP_Inverter_Init(void)
{
    /* RS485 默认为接收模式 */
    RS485_SetRx();

    /* 使用阻塞式收发, 禁用UART4中断防止Overrun错误 */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_ERR);

    /* 清空状态 */
    InvAckOK = 0;
    memset(&g_InvStatus, 0, sizeof(g_InvStatus));
}

/* ===================================================================
 *  BSP_Inverter_Send — 变频器控制接口
 *
 *  组装16字节帧并发送, 等待echo校验
 *
 *  cmd:     0x00=停机, 0x01=启动, 0x02=调频
 *  freq_hz: 目标频率 (Hz)
 *           停机时 freq_hz=0
 *           启动时 freq_hz=120 (初始频率)
 *           调频时 freq_hz=目标值 (120~320)
 * =================================================================== */
void BSP_Inverter_Send(uint8_t cmd, uint16_t freq_hz)
{
    /* 上限保护 */
    if (freq_hz > INV_FREQ_ABS_MAX) {
        freq_hz = INV_FREQ_ABS_MAX;
    }

    /* 组帧 */
    BuildFrame(cmd, freq_hz);

    /* 发送并校验echo */
    bool ok = SendAndVerifyEcho();

    /* 更新全局状态 */
    g_InvStatus.last_cmd = cmd;
    g_InvStatus.last_freq_hz = freq_hz;

    (void)ok;  /* echo失败不阻塞, 上层通过 g_InvStatus.comm_ok 判断 */
}

/* ===================================================================
 *  BSP_Inverter_ReadStatus — 读取变频器状态
 *
 *  老师的变频板暂无主动状态回读功能,
 *  此函数返回最近一次通信的echo结果, 兼容上层调用
 * =================================================================== */
bool BSP_Inverter_ReadStatus(InvStatus_t *out)
{
    if (out) {
        *out = g_InvStatus;
    }
    return g_InvStatus.comm_ok;
}

/* ===================================================================
 *  UART 接收完成回调
 *
 *  UART4  = 变频器 — 当前使用阻塞式收发, 此回调仅作备用
 *  USART1 = RS485 调试串口 — 逐字节中断接收
 * =================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        /* 变频器通信当前使用阻塞式, 暂不需要中断回调 */
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
