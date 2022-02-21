/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef CAN_TxHeader; // 发送帧头
CAN_RxHeaderTypeDef CAN_RxHeader; // 接收帧头
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;
uint8_t can_rx_buffer[CAN_BUFFER_SIZE];

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = (5 - 1);
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /*##-2- Configure the CAN Filter ###########################################*/
  // sFilterConfig.FilterBank = 0;
  // sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  // sFilterConfig.FilterIdHigh = CAN_ID << 5;
  // sFilterConfig.FilterIdLow = 0 | CAN_ID_STD;
  // sFilterConfig.FilterMaskIdHigh = ((CAN_ID << 3) >> 16) & 0xffff;
  // sFilterConfig.FilterMaskIdLow = (CAN_ID << 3) & 0xffff | CAN_ID_EXT;
  // sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  // sFilterConfig.FilterActivation = ENABLE;
  // sFilterConfig.SlaveStartFilterBank = 14;
  sFilterConfig.FilterBank = 0;                     //滤波器编号
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0
  sFilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    while (1)
    {
    }
  }
  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    while (1)
    {
    }
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    while (1)
    {
    }
  }
  // __HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  /*##-5- Configure Transmission process #####################################*/
  CAN_TxHeader.StdId = CAN_ID;     // 标准帧ID
  CAN_TxHeader.RTR = CAN_RTR_DATA; // 帧类型: 数据帧
  CAN_TxHeader.IDE = CAN_ID_STD;   // 帧ID: 标准帧ID
  CAN_TxHeader.DLC = 2;
  CAN_TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    /* USER CODE BEGIN CAN1_MspInit 1 */

    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // int i = 0;
  uint8_t RxData[8];
  // CAN_Receive_IT()函数会关闭FIFO0消息挂号中断，因此我们需要重新打开
  // __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //重新开启FIF00消息挂号中断

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, RxData);
}

// can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
// len:数据长度(最大为8)
// msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
uint8_t CAN1_Send_Msg(uint8_t *msg, uint8_t len)
{
  uint8_t i = 0;
  uint32_t TxMailbox;
  uint8_t message[8];
  CAN_TxHeader.StdId = CAN_ID;     //标准标识符
  CAN_TxHeader.IDE = CAN_ID_STD;   //使用标准帧
  CAN_TxHeader.RTR = CAN_RTR_DATA; //数据帧
  CAN_TxHeader.DLC = len;
  for (i = 0; i < len; i++)
  {
    message[i] = msg[i];
  }
  if (HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, message, &TxMailbox) != HAL_OK) //发送
  {
    return 1;
  }
  //	while(HAL_CAN_GetTxMailboxesFreeLevel(&CAN1_Handler) != 3) {}
  return 0;
}

// can口接收数据查询
// buf:数据缓存区;
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
uint8_t CAN1_Receive_Msg(uint8_t *buf)
{
  uint32_t i;
  uint8_t RxData[8];

  if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 1)
  {
    return 0xF1;
  }

  if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &CAN_RxHeader, RxData) != HAL_OK)
  {
    return 0xF2;
  }
  for (i = 0; i < CAN_RxHeader.DLC; i++)
    buf[i] = RxData[i];
  return CAN_RxHeader.DLC;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
