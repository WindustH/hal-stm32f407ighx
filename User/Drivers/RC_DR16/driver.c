/**
 * @file driver.c
 * @brief DR16遥控器驱动实现文件
 *
 * 实现DR16遥控器的UART通信配置和控制消息解析功能
 */

#include "driver.h"
#include "main.h"

static UART_HandleTypeDef *huartx;         ///< UART对象指针
static u8 uart_rx_buffer[DMA_BUFFER_SIZE]; ///< UART接收缓冲区
static volatile rcCtrl_dr16 rc_ctrl;       ///< 遥控器控制信息

void rc_setup_uart_dr16(UART_HandleTypeDef *huart) {
  // 使用DMA接收数据到空闲状态
  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buffer, DMA_BUFFER_SIZE) !=
      HAL_OK) {
    Error_Handler();
  }
  huartx = huart;
}

void rc_update_ctrl_msg_dr16(UART_HandleTypeDef *huart, u16 size) {
  (void)size; // 避免未使用参数警告

  // 检查是否为配置的UART实例
  if (huart->Instance == huartx->Instance) {
    // 解析接收到的遥控器控制消息
    rc_ctrl_msg_parse_dr16(uart_rx_buffer, &rc_ctrl);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huartx, uart_rx_buffer, DMA_BUFFER_SIZE);
  }
}

volatile rcCtrl_dr16 *rc_get_ctrl_sig_dr16() { return &rc_ctrl; }