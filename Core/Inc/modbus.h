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

// Recv_MB.flag 接收数据时的状态转换，先匹配地址、再匹配类型码、再匹配data
#define ADD_MATCH 0    // 地址
#define FRAME_MATCH 2  // 帧长
#define TYPE_MATCH 3   // 类型码
#define DATA_BUFFER 4  // 数据区
#define PROCESS_OVER 6 // 处理结束
#define ERROR_PRO 11   // 错误

// Recv_MB.typecode 类型码
#define MOD_READ_LOOP 0x01       // 读线圈
#define MOD_READ_0_1 0x02        // 读离散量
#define MOD_READ_PARA 0x03       // 读保持寄存器（共36个字节）
#define MOD_READ_IN 0x04         // 读输入寄存器
#define MOD_WRITE_S_WINDING 0x05 // 写单个线圈
#define MOD_WRITE_PARA 0x10      // 写保持寄存器（共10个字节）
//#define MOD_WRITE_PARA      0x02    // 写保持寄存器（共10个字节）
#define ADR_CONTROL_ON_OFF 0xFF00 // 分合闸线圈	操作地址

#define MOD_ERROR_NOCOMMAND 0x01    // 异常码	不支持功能码
#define MOD_ERROR_WRONG_ADR 0x02    // 异常码	起始地址或 起始地址+寄存器数量 不OK
#define MOD_ERROR_WRONG_NUM 0x03    // 异常码	寄存器数量超出范围
#define MOD_ERROR_CANT_READ 0x04    // 异常码	不能读输入寄存器
#define MOD_ERROR_READ_PARA 0x05    // 异常码	不能读保持寄存器
#define MOD_ERROR_WRITE_PARA 0x06   // 异常码	不能写保持寄存器
#define MOD_ERROR_WRITE_EEPROM 0x0A // 异常码	不能写EEPROM

// typecode 类型码
#define TC_READ_BIT 0x01        // 读线圈寄存器
#define TC_READ_ONLY_BIT 0x02   // 读离散输入寄存器
#define TC_READ_2BYTE 0x03      // 读保持寄存器
#define TC_READ_ONLY_2BYTE 0x04 // 读输入寄存器
#define TC_WRITE_BIT 0x05       // 写单个线圈寄存器
#define TC_WRITE_2BYTE 0x06     // 写单个保持寄存器
#define TC_WRITE_MUL_BIT 0x0F   // 写多个线圈寄存器
#define TC_WRITE_MUL_2BYTE 0x10 // 写多个保持寄存器

#define FRAME_PENDING 0xEE      // 帧长待定
#define FRAME_ERROR 0xFF        // 未知类型码，帧长返回ERROR
typedef struct
{
    uint8_t add;                 // 地址
    uint8_t typecode;            // 类型码
#define BUFFER_SIZE 64           // buffer大小，单位Byte
    uint8_t buffer[BUFFER_SIZE]; // 缓冲区
    // buffer[0]&buffer[1] 拼接形成寄存器起始地址
    // buffer[2]&buffer[3] 拼接形成寄存器输入数量
    uint16_t frame;     // 帧长 (不计算地址、类型码、校验码)
    uint16_t start_adr; // 寄存器起始地址
    uint16_t data_num;  // 寄存器数量
    uint8_t count;      // 通讯？       ？？？
    uint8_t flag;       // 通讯标志位  ？？？
    uint8_t length;
} struct_modbus; // 发送、接收的缓冲区

typedef struct 
{
    uint8_t     *bit_reg[16];
    uint16_t    *dual_byte_reg[16];
}modbus_register;


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
uint8_t Parse_Typecode(uint8_t typecode);
void Cancel_last_type(void);

void Over_Time_Pro(void);
void Send_Ready(void);
void Cal_Checkout_and_Send(void);
void return_WRONG(uint8_t err, uint8_t err_num);

void CRC_Check_On_Rcv(void);

void Recv_Wrong_data_Process(void);

/* USER CODE END Prototypes */

#endif /*__ MODBUS_H__ */
