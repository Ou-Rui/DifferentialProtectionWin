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

// 接收数据时的状态转换，先匹配地址、再匹配类型码、再匹配data
#define ADD_MATCH 0   //地址
#define FRAME_MATCH 2 //帧长
#define TYPE_MATCH 3  //类型码
#define DATA_BUFFER 4 //数据区

#define PROCESS_OVER 6 //处理结束
#define ERROR_PRO 11   //错误

#define BUFFER_SIZE 64 // buffer大小，单位Byte

// 类型码
#define MOD_READ_LOOP 0x01 //读线圈
#define MOD_READ_0_1 0x02  //读离散量
//#define	MOD_READ_0_1		0x33	//读离散量
#define MOD_READ_PARA 0x03       //读保持寄存器（共36个字节）
#define MOD_READ_IN 0x04         //读输入寄存器
#define MOD_WRITE_S_WINDING 0x05 //写单个线圈
#define MOD_WRITE_PARA 0x10      //写保持寄存器（共10个字节）
//#define MOD_WRITE_PARA      0x02    //写保持寄存器（共10个字节）
#define ADR_CONTROL_ON_OFF 0xFF00 //分合闸线圈	操作地址

typedef struct
{
    uint8_t add;                 // 地址
    uint8_t typecode;            // 类型码
    uint8_t buffer[BUFFER_SIZE]; // 缓冲区
    //buffer[0]&buffer[1] 拼接形成寄存器起始地址
    //buffer[2]&buffer[3] 拼接形成寄存器输入数量
    uint16_t frame;     // 帧长
    uint16_t start_adr; // 寄存器起始地址
    uint16_t data_num;  // 寄存器数量
    uint8_t count;      // 通讯？       ？？？
    uint8_t flag;       // 通讯标志位  ？？？
    uint8_t length;
} struct_modbus; // 发送、接收的缓冲区

#define h_MAX_BUG_LEN 60 //收发最大数据区

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void Buffer_int(void);              // 初始化收发缓冲区、串口任务结构体
void Modbus_onReceive_IT(void);     // Modbus，串口中断接收到数据时调用
void Modbus_OnSend_IT(void);        // Modbus，串口中断发送数据时调用

/* USER CODE END Prototypes */

#endif /*__ MODBUS_H__ */
