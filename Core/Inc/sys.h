#ifndef __SYS_H
#define __SYS_H
#include "stm32f103xe.h"
#include "stdint.h"
typedef struct tagPublicPara // 设备信息
{
    // uint8_t Real_Mode;     // 显示哪个屏幕
    // uint8_t SetShowTime;   // 切换时间
    // uint16_t Comm_Mode;    // 通讯方式
    uint8_t addr_rs485;      // 485通讯地址
    // uint8_t BaudRate;      // 485通讯波特率
    // uint8_t Add_TCP;       // 以太网通讯地址
    // uint8_t BaudRate_TCP;  // 以太网通讯波特率
    // uint8_t Add_FYJ;       // 防越级地址
    // uint8_t LCD_CON;       // 液晶背景灯控制
    // uint8_t LCD_CON_Time;  // 液晶背景灯点亮时间
    // uint32_t AccSaveCount; // 总共存储的次数
    // uint16_t Soft_Version; // 暂定版本号

    // uint8_t PHY_connect_count;
    // uint8_t TCP_Conn_exam;
    // uint8_t TCP_Interrupt_Flag;

    // uint16_t START_UPDATE;      // 从什么开始更新？？
    // uint8_t ACC_Lose_Time;
} PublicPara;

extern PublicPara Device;

void System_Init(void);

#endif
