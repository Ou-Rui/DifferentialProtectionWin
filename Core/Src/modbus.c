#include "modbus.h"

struct_modbus Recv_MB; // 接收缓冲区
struct_modbus Send_MB; // 发送缓冲区

void Buffer_int(void)
{
    uint8_t i;

    Recv_MB.add = 0;      // 地址
    Recv_MB.typecode = 0; // 类型码

    for (i = 0; i < h_MAX_BUG_LEN; i++)
    {
        Recv_MB.buffer[i] = 0; //缓冲区清零
    }
    Recv_MB.frame = 0;     //帧长
    Recv_MB.start_adr = 0; //起始寄存器地址
    Recv_MB.data_num = 0;  //寄存器数量
    Recv_MB.count = 0;     //通讯计数器
    Recv_MB.flag = 0;      //通讯标志
                           //  在TIRIdef.h中定义的ADD_MATCH为0
    Send_MB.add = 0;       //地址
    Send_MB.typecode = 0;  //类型码
    for (i = 0; i < h_MAX_BUG_LEN; i++)
    {
        Send_MB.buffer[i] = 0; //缓冲区清零
    }
    Send_MB.frame = 0;     //帧长
    Send_MB.start_adr = 0; //起始寄存器地址
    Send_MB.data_num = 0;  //寄存器数量
    Send_MB.count = 0;     //通讯计数器
    Send_MB.flag = 0;      //通讯标志

    ust.over_count = 0;                 //超时计数器
    ust.last_type = 0;                  //取消待执行事件
    ust.resend_type = 0;                //重发类型码
    ust.task_state = NONE_COMM_MSG;     //任务状态	串口任务处理模块
    ust.close_count = 0;                //合闸计数器
    ust.cut_count = 0;                  //分闸计数器
    ust.ready_task = 0;                 //就绪任务		分各种情况可以回传数据
    ust.wait_task = 0;                  //等待任务		分合闸、分闸、复位、存储数据等.
    ust.wait_task_main = NONE_COMM_MSG; //未定
    ust.task_main_flag = NONE_COMM_MSG; //执行结果未定
}

void Modbus_onReceive_IT()
{
    uint8_t tmp_Recv;
    HAL_UART_Receive(&huart2, &tmp_Recv, 1, 1000); //读取接收到的数据，每次读1Byte
    if (Device.START_UPDATE == TIRI_Update)        // 意义不明
    {
    }
    else
    {
        switch (Recv_MB.flag)
        {
        case ADD_MATCH: // 0，匹配地址
            if (Device.Add_Comm == tmp_Recv)
            {
                ust.task_state = RCV_PRO_MSG; // 设置串口状态：通讯接收消息中
                ust.over_count = 0;           // 超时计数器
                Recv_MB.add = tmp_Recv;       // 记录下地址，预留
                Recv_MB.flag = TYPE_MATCH;    // 下一步：匹配类型码
            }
            break;
        case TYPE_MATCH: // 3，匹配类型码
            Recv_MB.typecode = tmp_Recv;
            Recv_MB.frame = getFrame(Recv_MB.typecode); // 根据类型码，获取帧长
            if (Recv_MB.frame >= 60)                    // 帧长超过60，非法，状态重置
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

            if ((Recv_MB.typecode == MOD_WRITE_PARA) && (Recv_MB.count == 5)) // 判断帧长为什么不放在typecode里？
            {
                //Recv_MB.frame+=((uint8_t)(Recv_MB.buffer[3]>>1)-2);
                // Recv_MB.frame = if_frame_right1(Recv_MB.buffer[4]);
                Recv_MB.frame = Recv_MB.buffer[4] + 5;              // ？？？替换了if_frame_right1()
                // data中的存首地址和长度？根据长度决定帧长？
            }
            if ((Recv_MB.typecode == MOD_READ_PARA) || (Recv_MB.typecode == MOD_WRITE_S_WINDING)) // ？？？
            {
                Recv_MB.frame = 4;
            }

            //如果帧长度为0，则此时接收的数据是校验码
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
    }
}

void Modbus_OnSend_IT()
{
    switch (Send_MB.flag)
    {
    case ADD_MATCH:
        //地址
        break;
    case TYPE_MATCH:
        //类型码
        //UDR1=Send_MB.typecode;
        // USART_SendData(USART2, Send_MB.typecode);
        HAL_UART_Transmit(&huart2, &Send_MB.typecode, 1, 0xffff);
        ust.resend_type = Send_MB.typecode; //重发的类型码
        Send_MB.count = 0;
        Send_MB.flag = DATA_BUFFER;
        //while((UCSR1A&0x40)==0); //等待发送结束
        break;
    case DATA_BUFFER:
        //数据区
        //UDR1=Send_MB.buffer[Send_MB.count];
        // USART_SendData(USART2, Send_MB.buffer[Send_MB.count]);
        HAL_UART_Transmit(&huart2, &Send_MB.buffer[Send_MB.count], 1, 0xffff);
        Send_MB.count++;
        if (Send_MB.count >= Send_MB.frame + 2)
        {
            Send_MB.flag = PROCESS_OVER;
        }
        //while((UCSR1A&0x40)==0); //等待发送结束
        break;
    case PROCESS_OVER: //处理结束
        Send_MB.flag = ADD_MATCH;
        Recv_MB.flag = ADD_MATCH;
        // while (USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET); //等待发送结束
        RS485_RE_Mode();
        // USART_ITConfig(USART2, USART_IT_TXE, DISABLE); //禁止串口发送中断
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);    //禁止串口发送中断
        ust.task_state = NONE_COMM_MSG; // 串口任务：回到等待状态
        ust.over_count = 0;             // 超时计数器清零
        break;
    default:
        Receive_Wrong_Pro();
        break;
    }
}

// 输入类型码，返回一帧的帧长
// 如果功能码未知，则返回0xFF，如果是需要进一步判断寄存器的，那么返回0xEEEE
uint8_t getFrame(uint8_t type)
{
    switch (type)
    {
        //	case	MOD_READ_LOOP:
        //读线圈
        //		case	MOD_READ_IN:
        //读输入寄存器
        //		case	MOD_READ_0_1:
        //读离散量
    case MOD_WRITE_S_WINDING:
        //写单个线圈
    case MOD_READ_PARA:
        //读保持寄存器
        return 0x04;
        //数据区长度为4，连校验码长度共6个
        break;
    //case	MOD_WRITE_S_WINDING:
    //		//写保持寄存器
    //		return	0xEEEE;
    //		break;
    case MOD_WRITE_PARA: //写保持寄存器
        return 0x07;     //单个数据，数据区长度为38，连校验码长度共40个(2字节起始寄存器地址，2字节寄存器数量，一字节写入数据的总字节数)
        break;
    default: //未知功能码
        return 0xFF;
        break;
    }
}

void Receive_Wrong_Pro(void)
{
    ust.wait_task_main = BUFF_INIT_MSG; // 串口任务主状态（主程序中使用）：串口初始化
    uart2_init(Device.BaudRate);
    ust.task_state = WAI_WRO_PRO_MSG; // 串口任务状态：出错后等待串口复位
    RS485_RE_Mode();
}



