# DifferentialProtectionWin

- windows环境开发，差动保护项目ARM侧程序。使用STM32实现 UART + CAN 通信
- 开发板芯片型号：STM32F103ZET6

### HAL库
#### USART
- 清除标志位：USART_ClearFlag(USART2, USART_FLAG_TC); --> __HAL_UART_CLEAR_FLAG(&huart2, USART_FLAG_TC);
- 发送数据：USART_SendData(USART2, Device.Add_Comm); --> HAL_UART_Transmit(&huart2, &Device.Add_Comm, 1, 1000);
- 开启中断：USART_ITConfig(USART2, USART_IT_TXE, ENABLE); --> __HAL_UART_ENABLE_IT(&huart2, USART_IT_TXE);



### RS485
使用UART2作为485串口
1. `main()`中调用 `MX_USART2_UART_Init()`初始化串口 收发模式
2. `main()`中调用 `HAL_UART_Receive_IT()`该函数检查USART是否正忙，如果不忙则配置参数、使能中断等操作


### 开发日志
#### TODO
- 配一个定时器中断

#### 疑问
- `Judge_Comm_Work1()`中，为什么`Device.BaudRate == 0`属于正常情况

#### 2022/02/17
- 拆分了modbus_main.c + modbus_function.c

#### 2022/02/16
- 增加了sys.h，用于位操作
- `Serial_MSG`初步移植完成
- 编译好像有问题。。。

#### 2022/02/14
- 正在移植 `uint8_t Serial_MSG(void)`，该函数在定时器中断中调用


