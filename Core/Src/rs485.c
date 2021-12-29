#include "rs485.h"


// 485初始化，目前使用USART2
void RS485_Init() {
    MX_USART2_UART_Init();
}

// 485接收模式，RE低电平有效
void RS485_RE_Mode() {
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
}

// 485发送模式，DE高电平有效
void RS485_DE_Mode() {
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
}

// RS485接收时调用
void RS485_onReceive() {
    RS485_DE_Mode();
	HAL_UART_Transmit(&huart2, usart2RxBuffer, USART2_RX_BUFFER_SIZE, 1000);
    RS485_RE_Mode();
}