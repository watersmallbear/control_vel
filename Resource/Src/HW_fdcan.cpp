/**
 *******************************************************************************
 * @file      :HW_fdcan.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_fdcan.hpp"
#include "stdint.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static FDCAN_RxHeaderTypeDef rx_header1, rx_header3;
static uint8_t can1_rx_data[8], can3_rx_data[8];
uint32_t pTxMailbox;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void FdcanFilterInit(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo) {

  FDCAN_FilterTypeDef filter_config{
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterID1 = 0x000,
      .FilterID2 = 0x000,
      .RxBufferIndex = 0,
      .IsCalibrationMsg = 0,
  };
  filter_config.FilterConfig = fifo;

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/


/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/


/**
 * @brief   向can总线发送数据，抄官方的
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文
 * @param	len为发送数据长度（字节数）
 * @retval  none
 * @note    主控发送都是len=8字节，再加上帧间隔3位，理论上can总线1ms最多传输9帧
 **/
void FdcanSendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id,
                  uint8_t len) {
  FDCAN_TxHeaderTypeDef TxMessageHeader = {0};

  TxMessageHeader.Identifier = id;                // 32位ID
  TxMessageHeader.IdType = FDCAN_STANDARD_ID;     // 标准ID
  TxMessageHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
  TxMessageHeader.DataLength = len;               // 数据长度
  TxMessageHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxMessageHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
  TxMessageHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
  TxMessageHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
  TxMessageHeader.MessageMarker = 0;
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessageHeader, msg) != HAL_OK) {
  }
}
