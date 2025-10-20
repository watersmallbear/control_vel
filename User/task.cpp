#include "main.h"
#include "task.hpp"
#include "HW_fdcan.hpp"
#include "tim.h"
#include "GM6020.hpp"
#include "cmath"
#include "PID.hpp"
#include "iwdg.h"

uint32_t tick = 0;
uint8_t can1_tx_data[8] = {0};
uint8_t can1_rx_data[8] = {0};
uint8_t can2_tx_data[8] = {0};
uint8_t can2_rx_data[8] = {0};
uint16_t current = 5000;
float target_vel = 0;
uint32_t test;

static FDCAN_RxHeaderTypeDef rx_header1;

GM6020 Motor1;
Pid Vel_Pid;

void MainInit() {
 
  Motor1.Init(4);
  Vel_Pid.Init(1000,0,2,0.001);
    
  FdcanFilterInit(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
  HAL_TIM_Base_Start_IT(&htim6);

}

void MainTask() {
  HAL_IWDG_Refresh(&hiwdg1);
  tick ++;

  target_vel = sin((float)tick / 2000.0) * 5;
  
  Vel_Pid.Control(target_vel,Motor1.vel());
  Motor1.setInput(Vel_Pid.Result());
  Motor1.encode(can1_tx_data);
  

  FdcanSendMsg(&hfdcan1, can1_tx_data, 0x1FE, 8);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  if (htim == &htim6){
        MainTask();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan == &hfdcan1) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header1,can1_rx_data) ==HAL_OK) // 获得接收到的数据头和数据
    {
      if (rx_header1.Identifier == 0x208) { // 帧头校验
        Motor1.decode(can1_rx_data);
      }
    }
  } else if (hfdcan == &hfdcan2) {
  }
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                 0); // 再次使能FIFO0接收中断
}

