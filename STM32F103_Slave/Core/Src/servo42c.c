// 42C 电机发送命令函数
#include "servo42c.h"

extern UART_HandleTypeDef huart1; // 声明 UART 句柄
void Servo_SendCommand(uint8_t func, uint8_t *data, uint8_t len)
{
    uint8_t buf[8];// 发送缓冲区
    uint8_t sum = 0; // 校验和

    buf[0] = SERVO_ADDR;// 42C 电机地址
    buf[1] = func;// 功能字
    
    for (uint8_t i = 0; i < len; i++)
    {
        buf[2 + i] = data[i];
        sum += data[i];// 加上数据字节
    }
    sum += buf[0];// 加上地址字节
    sum += buf[1];// 加上功能字节
    buf[2 + len] = sum;// 校验和字节
    HAL_UART_Transmit(&huart1, buf, 2 + len + 1, 100);// 发送数据
}
// 42C 电机使能函数
void Servo_Enable(void) 
{
    uint8_t dummy[3] = {0}; // 接收应答的缓冲区
    uint8_t flush_tmp = 0; // 清空 UART 启动残留字节
    while (HAL_UART_Receive(&huart1, &flush_tmp, 1, 5) == HAL_OK) {}
    uint8_t data_mode[1] = {0x02}; // 0x02 = CR_UART 串口控制模式
    Servo_SendCommand(0x82, data_mode, 1); // 设置工作模式（关键：默认是脉冲模式，串口运动命令不驱动电机）
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 记录模式回复
    HAL_Delay(2000); // 模式切换等待加长到 2 秒
    while (HAL_UART_Receive(&huart1, &flush_tmp, 1, 5) == HAL_OK) {} // 清模式切换残留
    
    uint8_t data_en[1] = {0x02}; // 0x02 = Hold 一直使能（排除悬空 EN 引脚影响）
    Servo_SendCommand(0x85, data_en, 1); // 设置 EN 引脚有效电平
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 记录 EN 引脚回复
    HAL_Delay(500); // 等 EN 设置生效
    while (HAL_UART_Receive(&huart1, &flush_tmp, 1, 5) == HAL_OK) {} // 清残留

    uint8_t data[1] = {0x01}; // 使能数据
    Servo_SendCommand(SERVO_FUNC_ENABLE, data, 1); // 发送使能命令
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 记录使能回复
    HAL_Delay(500);
    while (HAL_UART_Receive(&huart1, &flush_tmp, 1, 5) == HAL_OK) {} // 清使能残留字节
 // 使能等待加长到 500ms
}
// 42C 置零命令：把当前编码器位置设为零（手动回零用）
void Servo_SetZero(void)
{
    uint8_t dummy[3] = {0}; // 接收应答的缓冲
    uint8_t flush_tmp = 0;  // 清 UART 残留字节
    Servo_SendCommand(SERVO_FUNC_SET_ZERO, NULL, 0); // 发送置零命令
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 读 3 字节应答
    while (HAL_UART_Receive(&huart1, &flush_tmp, 1, 5) == HAL_OK) {} // 清残留
}

// 42C 电机停止函数
void Servo_Stop(void)
{
    uint8_t dummy[3] = {0}; // 接收应答的缓冲区
    Servo_SendCommand(SERVO_FUNC_STOP, NULL, 0); // 发送停止命令
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 等待回复
}
// 42C 电机速度控制函数
void Servo_SetSpeed(uint8_t dir, uint8_t speed)
{
    uint8_t dummy[3] = {0}; // 接收应答的缓冲区
    uint8_t val = (dir << 7) | (speed & 0x7F); // 方向+速度拼成一个字节
    Servo_SendCommand(SERVO_FUNC_SPEED, &val, 1); // 发送速度控制命令
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 等待回复
}
// 42C 电机读取编码器函数
int32_t Servo_ReadEncoder(void)
{
    uint8_t rx[8] = {0}; // 接收缓冲区（电机 0x30 回复固定 8 字节）
    uint8_t b = 0; // 单字节临时变量
    uint32_t t0 = HAL_GetTick(); // 起始时刻，用于总超时
    uint8_t sum = 0; // 校验和

    uint8_t flush_tmp = 0; // 清空缓冲区残留字节
    while (HAL_UART_Receive(&huart1, &flush_tmp, 1, 5) == HAL_OK) {} // 清掉上次0xFD的"运行完成"回复

    Servo_SendCommand(SERVO_FUNC_READ_ENC, NULL, 0); // 发送读取编码器命令

    // 逐字节找帧头 0xE0，丢弃残留垃圾字节（比一次性读 8 字节更抗错位）
    while (1)
    {
        if (HAL_UART_Receive(&huart1, &b, 1, 10) == HAL_OK)
        {
            if (b == SERVO_ADDR) { rx[0] = b; break; }
        }
        if (HAL_GetTick() - t0 > 200) // 200ms 内没找到帧头，超时
        {
            return -1;
        }
    }

    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &rx[1], 7, 100); // 帧头后读剩余 7 字节

    if (status != HAL_OK)
        return -1; // 没收到 7 字节，超时

    // CRC 校验：前 7 字节求和取低 8 位，应等于第 8 字节
    sum = 0;
    for (uint8_t i = 0; i < 7; i++) sum += rx[i];
    if (sum != rx[7])
    {
        return -1;
    }

    uint16_t value = (uint16_t)((rx[5] << 8) | rx[6]);
    int32_t carry = (int32_t)(((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | (uint32_t)rx[4]);
    return carry * 65536 + (int32_t)value; // 返回完整多圈位置（carry*16384+value）
}

// 42C 电机位置控制函数（相对位置模式，0xFD）
void Servo_SetPosition(uint8_t dir, uint8_t speed, int32_t step)
{
    uint8_t dummy[3] = {0}; // 接收应答的缓冲区
    uint8_t data[5] = {0};
    data[0] = (dir << 7) | (speed & 0x7F); // 方向(bit7) + 速度(低7位)
    data[1] = (step >> 24) & 0xFF;         // step 高字节（大端）
    data[2] = (step >> 16) & 0xFF;
    data[3] = (step >> 8) & 0xFF;
    data[4] = step & 0xFF; // step 低字节
    Servo_SendCommand(SERVO_FUNC_POS, data, 5); // 发送位置控制命令
    HAL_UART_Receive(&huart1, dummy, 3, 100); // 等待回复
}

