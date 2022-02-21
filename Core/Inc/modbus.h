#ifndef __MODBUS_H__
#define __MODBUS_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "usart.h"
#include "typedef.h"
#include "rs485.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define OVER_TIME 50 // 超时时限


// // Recv_MB.typecode 类型码
// #define MOD_READ_LOOP 0x01       // 读线圈
// #define MOD_READ_0_1 0x02        // 读离散量
// #define MOD_READ_PARA 0x03       // 读保持寄存器（共36个字节）
// #define MOD_READ_IN 0x04         // 读输入寄存器
// #define MOD_WRITE_S_WINDING 0x05 // 写单个线圈
// #define MOD_WRITE_PARA 0x10      // 写保持寄存器（共10个字节）
// //#define MOD_WRITE_PARA      0x02    // 写保持寄存器（共10个字节）
// #define ADR_CONTROL_ON_OFF 0xFF00 // 分合闸线圈	操作地址

// typecode 类型码
#define TC_READ_BIT 0x01        // 读线圈寄存器
#define TC_READ_ONLY_BIT 0x02   // 读离散输入寄存器
#define TC_READ_2BYTE 0x03      // 读保持寄存器
#define TC_READ_ONLY_2BYTE 0x04 // 读输入寄存器
#define TC_WRITE_BIT 0x05       // 写单个线圈寄存器
#define TC_WRITE_2BYTE 0x06     // 写单个保持寄存器
#define TC_WRITE_MUL_BIT 0x0F   // 写多个线圈寄存器
#define TC_WRITE_MUL_2BYTE 0x10 // 写多个保持寄存器
// 异常码，异常时的buffer[0]
#define MOD_ERROR_NOCOMMAND 0x01    // 异常码	不支持功能码
#define MOD_ERROR_WRONG_ADDR 0x02    // 异常码	起始地址或 起始地址+寄存器数量 不OK
#define MOD_ERROR_WRONG_NUM 0x03    // 异常码	寄存器数量超出范围
#define MOD_ERROR_CANT_READ 0x04    // 异常码	不能读输入寄存器
#define MOD_ERROR_READ_PARA 0x05    // 异常码	不能读保持寄存器
#define MOD_ERROR_WRITE_PARA 0x06   // 异常码	不能写保持寄存器
#define MOD_ERROR_WRITE_EEPROM 0x0A // 异常码	不能写EEPROM



#define FRAME_PENDING 0xEE // 帧长待定
#define FRAME_ERROR 0xFF   // 未知类型码，帧长返回ERROR
typedef struct
{
    // 前三项固定顺序，供CRC使用
    uint8_t add;                 // 地址
    uint8_t typecode;            // 类型码
#define MB_BUFFER_SIZE 64           // buffer大小，单位Byte
    uint8_t buffer[MB_BUFFER_SIZE]; // 缓冲区
    // buffer[0]&buffer[1] 拼接形成寄存器起始地址
    // buffer[2]&buffer[3] 拼接形成寄存器输入数量
    uint8_t length;     // 数据长度，用于多寄存器读写中
    uint16_t frame;     // 帧长 (不计算地址、类型码、校验码)
    uint16_t start_addr; // 寄存器起始地址
    uint16_t data_num;  // 寄存器数量，用于多个寄存器读写中
    uint8_t count;      // 通讯？       ？？？

// Recv_MB.flag 接收数据时的状态转换，先匹配地址、再匹配类型码、再匹配data
#define ADD_MATCH 0    // 地址
#define TYPE_MATCH 1   // 类型码
#define DATA_BUFFER 2  // 数据区
#define FRAME_MATCH 3  // 帧长
#define PROCESS_OVER 6 // 处理结束
#define ERROR_PRO 11   // 错误
    uint8_t flag;       // 通讯标志位  ？？？

} struct_modbus; // 发送、接收的缓冲区

typedef struct
{
#define MODBUS_REG_SIZE 16
    uint16_t bit_reg_num;
    uint8_t *bit_reg[MODBUS_REG_SIZE];
    uint16_t dual_byte_reg_num;
    uint16_t *dual_byte_reg[MODBUS_REG_SIZE];
} modbus_register;

#define h_MAX_BUG_LEN 60 // 收发最大数据区

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
extern struct_modbus Recv_MB; // 接收缓冲区
extern struct_modbus Send_MB; // 发送缓冲区

extern modbus_register Reg_MB;

extern uint16_t ia;
extern uint16_t ib;
extern uint16_t ic;

void Buffer_int(void); // 初始化收发缓冲区、串口任务结构体
void Serial_Init(void);
void Modbus_Init_Reg(void);

// 供其他模块调用的接口函数
void Modbus_OnReceive_IT(void); // Modbus，串口中断接收到数据时调用
void Modbus_OnSend_IT(void);    // Modbus，串口中断发送数据时调用
void Modbus_Timer_Process(void);
void Modbus_Main_Process(void);

// private function
uint16_t CRC16(const uint8_t *pbuf, int len);

// 接收
uint8_t Parse_Typecode(uint8_t typecode);
void Cancel_last_type(void);
void CRC_Check_On_Rcv(void);
void Over_Time_Pro(void);

//发送
void Process_Command_and_Reply(void);
void Reply_Read_2Byte(void);
void Init_Send_MB(void);
void Cal_CRC_and_Send(void);
void Load_Error(uint8_t err_code);

void Recv_Wrong_data_Process(void);

/* USER CODE END Prototypes */

#endif /*__ MODBUS_H__ */
