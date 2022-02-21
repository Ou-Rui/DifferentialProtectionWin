#include "rs485.h"
#include "usart.h"

// 485初始化，目前使用USART2
void RS485_Init(void)
{
    MX_USART2_UART_Init();
}

void RS485_Reset(void)
{
    ust.wait_task_main = BUFF_INIT_MSG; // 串口任务主状态（主程序中使用）：串口初始化
    MX_USART2_UART_Init();
    ust.task_state = WAI_WRO_PRO_MSG; // 串口任务状态：出错后等待串口复位
    RS485_RE_Mode();
}

GPIO_PinState RS485_Get_Mode()
{
    return HAL_GPIO_ReadPin(RS485_RE_Port, RS485_RE_Pin);
}

// 485接收模式，RE低电平有效
void RS485_RE_Mode(void)
{
    HAL_GPIO_WritePin(RS485_RE_Port, RS485_RE_Pin, RS485_RE_LEVEL);
}

// 485发送模式，DE高电平有效
void RS485_DE_Mode(void)
{
    HAL_GPIO_WritePin(RS485_RE_Port, RS485_RE_Pin, RS485_DE_LEVEL);
}

// // RS485接收时调用
// void RS485_onReceive(void)
// {
//     RS485_DE_Mode();
//     HAL_UART_Transmit(&huart2, usart2RxBuffer, USART2_RX_BUFFER_SIZE, 1000);
//     RS485_RE_Mode();
// }