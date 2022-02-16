#include "modbus.h"
#include "sys.h"
#include "stm32f1xx_hal_usart.h"

struct_modbus Recv_MB; // 接收缓冲区
struct_modbus Send_MB; // 发送缓冲区

/* Table of CRC values for high-order byte */
static const unsigned char array_crc_low[] =
    {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};
/* Table of CRC values for low-order byte */
static const unsigned char array_crc_high[] =
    {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
        0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
        0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
        0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
        0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
        0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
        0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
        0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
        0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
        0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
        0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
        0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
        0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

uint16_t CRC16(const uint8_t *pbuf, int len)
{
    uint8_t crc_high, crc_low;
    uint32_t index;
    crc_high = crc_low = 0xff;
    while (len-- > 0)
    {
        index = crc_low ^ *pbuf++;
        crc_low = crc_high ^ array_crc_low[index];
        crc_high = array_crc_high[index];
    }
    return (crc_low + (crc_high << 8));
}

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

// 接收数据的中断处理
void Modbus_OnReceive_IT()
{
    uint8_t tmp_Recv;
    HAL_UART_Receive(&huart2, &tmp_Recv, 1, 1000); // 读取接收到的数据，每次读1Byte
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
                // Recv_MB.frame+=((uint8_t)(Recv_MB.buffer[3]>>1)-2);
                //  Recv_MB.frame = if_frame_right1(Recv_MB.buffer[4]);
                Recv_MB.frame = Recv_MB.buffer[4] + 5; // ？？？替换了if_frame_right1()
                // data中的存首地址和长度？根据长度决定帧长？
            }
            if ((Recv_MB.typecode == MOD_READ_PARA) || (Recv_MB.typecode == MOD_WRITE_S_WINDING)) // ？？？
            {
                Recv_MB.frame = 4;
            }

            // 如果帧长度为0，则此时接收的数据是校验码
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
    // case	MOD_WRITE_S_WINDING:
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
    // uart2_init(Device.BaudRate);
    MX_USART2_UART_Init();
    ust.task_state = WAI_WRO_PRO_MSG; // 串口任务状态：出错后等待串口复位
    RS485_RE_Mode();
}

// 定时器中断中执行，串行通讯的准备工作
uint8_t Serial_MSG(void)
{
    if (Device.START_UPDATE == TIRI_Update)
    {
        return 0;
    }
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
        else
        {
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
        // 通讯接收结束消息	接收结束就设置此状态鞒绦蛑信卸辖榆码启动发送
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

// 查看停机的执行结果
// 本台低速启动失败判断处理	  +1表示  20mS
// 执行条件:	启动命令持续一个固定的时间,时间到后
//		检测执行结果
//		执行结果正确,则撤消该信号
//		执行结果错误,则判定为执行故障
// 取消待执行的命令
void Cancel_last_type(void)
{
    uint16_t static count = 0;
    if (ust.last_type != 0)
    {
        if (++count > 500)
        {
            count = 0;
            ust.last_type = 0; // 取消待执行事件
        }
    }
    else
    {
        count = 0;
    }
}

// 接收数据完毕，处理事件中，如分合闸复位保存参数等
// 0表示通讯设置正常
uint8_t Judge_Comm_Work1(void)
{
    // 波特率，疑问：为什么波特率=0属于正常？
    if (Device.BaudRate == 0)
    {
        return 0;
    }
    // 控制脚如果处于发送状态，属于错误
    if ((PAin(1)) == 1)
    {
        return 1;
    }
    return 0;
}

// 超时处理，应该将所有的计数器等恢复初始状态
void Over_Time_Pro(uint8_t volatile Ot_cnt)
{
    if ((++ust.over_count) >= Ot_cnt)
    {
        Receive_Wrong_Pro();
        ust.over_count = 0;
    }
}

//接收到错误数据，返回等待
void Recv_Wrong_data_Process(void)
{
    ust.task_state = NONE_COMM_MSG; // 回到等待
    Buffer_int();
    RS485_DE_Mode();
}

// 接收到数据的处理，包含CRC校验
void Recv_Data_Process(void)
{
    uint8_t volatile i;
    uint16_t crc, sum, xxx;
    i = Recv_MB.frame; // 取得帧长
    crc = CRC16(&Recv_MB.add, i + 2);
    sum = (Recv_MB.buffer[i]) + (((uint16_t)(Recv_MB.buffer[i + 1])) << 8); // 配合本crc校验，高低位互换

    if (sum == crc)
    {
        ust.task_state = RCV_OVER_MSG; // 转为接收结束
        RS485_DE_Mode();
        // enable_DE();
    }
    else
    {
        ust.task_state = RCV_WRO_MSG; // 校验失败处理
    }
    ust.over_count = 0; // 超时计数器清零
}

//接收数据的处理
void Send_Ready(void)
{
    switch (Recv_MB.typecode)
    {
    case MOD_READ_PARA:
        // 读保持寄存器
        // MOD_READ_PARA_Pro();
        break;
    case MOD_WRITE_PARA:
        // 写保持寄存器
        // if ((WM.pro.item.ContChk == OPEN) || (WM.pro.item.ContChk == CLOSE)) //近控能否设置参数？
        // {
        //     MOD_WRITE_PARA_Pro();
        // }
        // else
        // {
        //     return_WRONG(MOD_WRITE_PARA, MOD_ERROR_WRITE_PARA);
        //     Cal_Checkout_and_Send();
        // }
        break;
    case MOD_WRITE_S_WINDING:
        // 写单个线圈
        // MOD_WRITE_S_WINDING_Pro();
        break;
    default:
        // 接收正确
        return_WRONG(MOD_READ_IN, MOD_ERROR_NOCOMMAND); //对于不认识的功能码暂不处理，暂用04错误代替
        Cal_Checkout_and_Send();
        break;
    }
}

void Cal_Checkout_and_Send(void)
{
    uint8_t volatile sum, i;
    uint16_t crc, xxx;
    Send_MB.add = Device.Add_Comm; // 此AVR的地址
    i = Send_MB.frame;             // 取得帧长
    crc = CRC16(&Send_MB.add, i + 2);

    Send_MB.buffer[i] = crc & 0x00FF; // 先传低位
    Send_MB.buffer[i + 1] = crc >> 8; // 再传高位

    enable_DE();
    Send_MB.flag = TYPE_MATCH;     // 直接进入类型匹配模式，跳过地址模式
    ust.task_state = SEND_PRO_MSG; // 进入发送模式
    ust.over_count = 0;            // 超时计数器

    // 直接发送地址，因为不发送无法进入发送中断
    __HAL_UART_CLEAR_FLAG(&huart2, USART_FLAG_TC);
    HAL_UART_Transmit(&huart2, &Device.Add_Comm, 1, 1000);
    __HAL_UART_ENABLE_IT(&huart2, USART_IT_TXE);

    // USART_ClearFlag(USART2, USART_FLAG_TC);
    // USART_SendData(USART2, Device.Add_Comm);
    // USART_ITConfig(USART2, USART_IT_TXE, ENABLE); // 开启串口发送中断
}

void return_WRONG(uint8_t err, uint8_t err_num)
{
    // 返回	差错码
    Send_MB.typecode |= 0x80;
    // 返回	异常码
    Send_MB.buffer[0] = err_num;
    Send_MB.frame = 1;
    // 返回错误时数据区只有一个字节，即为错误err_num
}
