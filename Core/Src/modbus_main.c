#include "modbus.h"
#include "sys.h"
#include "stm32f1xx_hal_usart.h"

struct_modbus Recv_MB; // 接收缓冲区
struct_modbus Send_MB; // 发送缓冲区

void Buffer_int(void)
{
    uint8_t i;

    Recv_MB.add = 0;      // 地址
    Recv_MB.typecode = 0; // 类型码

    for (i = 0; i < h_MAX_BUG_LEN; i++)
    {
        Recv_MB.buffer[i] = 0; // 缓冲区清零
    }
    Recv_MB.frame = 0;        // 帧长
    Recv_MB.start_adr = 0;    // 起始寄存器地址
    Recv_MB.data_num = 0;     // 寄存器数量
    Recv_MB.count = 0;        // 通讯计数器
    Recv_MB.flag = ADD_MATCH; // 通讯标志, 初始应为地址匹配
    Send_MB.add = 0;          // 地址
    Send_MB.typecode = 0;     // 类型码
    for (i = 0; i < h_MAX_BUG_LEN; i++)
    {
        Send_MB.buffer[i] = 0; //缓冲区清零
    }
    Send_MB.frame = 0;        // 帧长
    Send_MB.start_adr = 0;    // 起始寄存器地址
    Send_MB.data_num = 0;     // 寄存器数量
    Send_MB.count = 0;        // 通讯计数器
    Send_MB.flag = ADD_MATCH; // 通讯标志

    ust.over_count = 0;                 // 超时计数器
    ust.last_type = 0;                  // 取消待执行事件
    ust.resend_type = 0;                // 重发类型码
    ust.task_state = NONE_COMM_MSG;     // 任务状态	串口任务处理模块
    ust.close_count = 0;                // 合闸计数器
    ust.cut_count = 0;                  // 分闸计数器
    ust.ready_task = 0;                 // 就绪任务		分各种情况可以回传数据
    ust.wait_task = 0;                  // 等待任务		分合闸、分闸、复位、存储数据等.
    ust.wait_task_main = NONE_COMM_MSG; // 未定
    ust.task_main_flag = NONE_COMM_MSG; // 执行结果未定
}

void Serial_Init(void)
{
    Buffer_int();
    MX_USART2_UART_Init();
    RS485_RE_Mode();
}

// 接收数据的中断处理
void Modbus_OnReceive_IT()
{
    // HAL_UART_Receive(&huart2,(uint8_t *)usart2_rx_buffer,1,1000); 
    uint8_t tmp_Recv = usart2_rx_buffer[0];
    switch (Recv_MB.flag)
    {
    case ADD_MATCH:                      // 0，匹配地址
        if (Device.Add_Comm == tmp_Recv) // 与Device地址匹配
        {
            ust.task_state = RCV_PRO_MSG; // 设置串口状态：通讯接收消息中
            ust.over_count = 0;           // 超时计数器
            Recv_MB.add = tmp_Recv;       // 记录下地址，预留
            Recv_MB.flag = TYPE_MATCH;    // 下一步：匹配类型码
        }
        break;
    case TYPE_MATCH: // 3，匹配类型码
        Recv_MB.typecode = tmp_Recv;
        Recv_MB.frame = Parse_Typecode(Recv_MB.typecode); // 根据类型码，获取帧长
        if (Recv_MB.frame == FRAME_ERROR)                 // 非法，状态重置
        {
            Recv_MB.flag = ADD_MATCH;       // 接收状态重置为匹配地址
            Recv_MB.frame = 0;              // 帧长置零
            ust.task_state = NONE_COMM_MSG; // 串口状态重置：无串口消息（空闲）
            ust.over_count = 0;             // 超时计数器
        }
        else
        {
            Recv_MB.flag = DATA_BUFFER; // 合法的帧长，下一步：接收data
            Recv_MB.count = 0;          // data的长度置零
        }
        break;
    case DATA_BUFFER:                             // 4，接收data
        Recv_MB.buffer[Recv_MB.count] = tmp_Recv; // 逐字节接收data到缓冲区
        Recv_MB.count++;

        if (Recv_MB.frame == FRAME_PENDING && Recv_MB.count == 5) // 如果帧长待定，此时可确认帧长 (写多个寄存器时进入该if)
        {
            // buffer[0] [1]组成寄存器首地址，buffer[2] [3]组成数据长度(单位: 2Byte), buffer[4]为后续数据长度
            uint16_t len_2byte = (Recv_MB.buffer[2] << 8) + Recv_MB.buffer[3];
            uint64_t tmp = len_2byte * 2 + 5; // 防止越界
            if (tmp <= 0xFFFF)
            {
                Recv_MB.frame = len_2byte * 2 + 5;
            }
            else
            {
                Receive_Wrong_Pro();
                break;
            }
        }

        // >= 帧长+2 说明数据和校验位都接收完了
        if (Recv_MB.count >= Recv_MB.frame + 2)
        {
            ust.task_state = RCV_DATA_MSG; // 串口任务状态：接收结束
            RS485_DE_Mode();               // 接收完成，切换成发送模式
            ust.over_count = 0;            // 超时计数器
            Recv_MB.count = 0;
            Recv_MB.flag = ADD_MATCH;
        }
        break;
    default:
        Receive_Wrong_Pro();
        break;
    }
    // 重新注册中断接收
    // __HAL_UART_CLEAR_OREFLAG(&huart2);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)usart2_rx_buffer, USART2_RX_BUFFER_SIZE);
}

// 发送数据的中断处理
void Modbus_OnSend_IT()
{
    switch (Send_MB.flag)
    {
    case ADD_MATCH:
        // 地址
        break;
    case TYPE_MATCH:
        // 类型码
        // USART_SendData(USART2, Send_MB.typecode);
        HAL_UART_Transmit(&huart2, &Send_MB.typecode, 1, 0xffff);
        ust.resend_type = Send_MB.typecode; //重发的类型码
        Send_MB.count = 0;
        Send_MB.flag = DATA_BUFFER;
        // while((UCSR1A&0x40)==0); //等待发送结束
        break;
    case DATA_BUFFER:
        // 数据区
        // USART_SendData(USART2, Send_MB.buffer[Send_MB.count]);                   // 发送数据
        HAL_UART_Transmit(&huart2, &Send_MB.buffer[Send_MB.count], 1, 0xffff); // 发送数据
        Send_MB.count++;
        if (Send_MB.count >= Send_MB.frame + 2)
        {
            Send_MB.flag = PROCESS_OVER;
        }
        // while((UCSR1A&0x40)==0); //等待发送结束
        break;
    case PROCESS_OVER: //处理结束
        Send_MB.flag = ADD_MATCH;
        Recv_MB.flag = ADD_MATCH;
        // while (USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET); //等待发送结束
        RS485_RE_Mode();
        // USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //禁止串口发送中断
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE); //禁止串口发送中断
        ust.task_state = NONE_COMM_MSG;              // 串口任务：回到等待状态
        ust.over_count = 0;                          // 超时计数器清零
        break;
    default:
        Receive_Wrong_Pro();
        break;
    }
}

// 定时器中断中执行，串行通讯的准备工作
void Serial_MSG(void)
{
    Cancel_last_type();
    switch (ust.task_state)
    {
    case NONE_COMM_MSG:
        // 无通讯消息，正常在此状态
        // 判断是否可以进行通讯??
        if (Judge_Comm_Work1() != 0)
        {
            Receive_Wrong_Pro();
        }
        break;
    case RCV_PRO_MSG:
        // 通讯接收消息中..	接收开始就设置此状态，接受完毕设置为接收结束
        Over_Time_Pro(OVER_TIME);
        break;
    case RCV_DATA_MSG:
        Recv_Data_Process();
        break;
    case RCV_OVER_MSG:
        // 通讯接收结束消息	接收结束就设置此状态，此时485为发送模式
        Send_Ready();
        Over_Time_Pro(OVER_TIME);
        break;
    case SEND_DO_MSG:
        // 通锻ㄑ妒挛锎碇?处理完毕移到 消息发送状态
        // 执行时必须保证超时处理
        // Wait_Process();
        break;
    case RCV_WRO_MSG:
        // 通讯中接收数据校验出错
        Recv_Wrong_data_Process();
        break;
    case SEND_PRO_MSG:
        // 通讯发送消⒅?....	发送结束设置为下状态，开始发送则设置此码
        Over_Time_Pro(OVER_TIME);
        break;
    case WAI_WRO_PRO_MSG:
        // 出错后等待串口复位
        break;
    default:
        Over_Time_Pro(OVER_TIME);
        break;
    }
}

//主程序执行串行通讯结果
void Serial_MSG_Main(void)
{
    switch (ust.wait_task_main)
    {
    case BUFF_INIT_MSG:
        //串行通讯初始化
        Serial_Init();
        break;
    default:
        break;
    }
    ust.wait_task_main = NONE_COMM_MSG;
}
