/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "rs485.h"
  /* USER CODE END Includes */

  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define USART2_RX_BUFFER_SIZE 1

  // 串口通讯任务结构体，描述串口状态
  typedef struct ust_comm_state
  {
    uint8_t over_count;       // 超时计数器
    uint8_t last_type;        // 上次类型码
    uint8_t last_patrol_type; // 上次巡查类型码
    uint8_t resend_type;      // 重发类型码

#define NONE_COMM_MSG 0x00   // 无通讯消息		正常时应该在此状态
#define RCV_PRO_MSG 0x01     // 通讯接收消息中..	接收开始就设置此状态，接受完毕设置为接收结束
#define RCV_OVER_MSG 0x02    // 通讯接收结束消息	接收结束就设置此状态，主程序中判断接受码启动发送
#define RCV_DATA_MSG 0x03    // 通讯接收结束消息	接收结束就设置此状态，主程序中判断接受码启动发送
#define SEND_DO_MSG 0x04     // 通讯通讯事物处理中	处理完毕转移到 消息发送状态
#define RCV_WRO_MSG 0x05     // 通讯中接收数据校验出错
#define SEND_PRO_MSG 0x06    // 通讯发送消息中.....	发送结束设置为下状态，开始发送则设置此码
#define SEND_OVER_MSG 0x07   // 通讯发送结束消息	主程序返回无通讯消息
#define WAI_WRO_PRO_MSG 0x08 // 出错后等待串口复位

    uint8_t task_state; // 任务状态	串口任务处理模块

    uint8_t close_count; // 合闸计数器
    uint8_t cut_count;   // 分闸计数器
    uint8_t ready_task;  // 就绪任务		分各种情况可以回传数据
    uint8_t wait_task;   // 等待任务		分合闸、分闸、复位、存储数据等

// 主程序中用
#define Comp_Para_MSG 0x01  // 比较数据
#define SAVE_PARA_MSG 0x02  // 存储数据
#define CLEAR_FAU_MSG 0x03  // 清除故障库
#define CLEAR_ACL_MSG 0x04  // 清除累计信息
#define SET_SYS_T_MSG 0x05  // 存储系统时间
#define GET_FAU_N_MSG 0x06  // 取出第N号故障
#define BUFF_INIT_MSG 0x07  // 串行通讯初始化
    uint8_t wait_task_main; // 等待任务(主程序中)	存储数据等
    uint8_t task_main_flag; // 主程序任务执行标志	存储数据等

  } usart_task; // 串口通讯任务
  /* USER CODE END Private defines */

  void MX_USART1_UART_Init(void);
  void MX_USART2_UART_Init(void);

  /* USER CODE BEGIN Prototypes */
  extern uint8_t usart2RxBuffer[USART2_RX_BUFFER_SIZE];
  extern usart_task ust;
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
