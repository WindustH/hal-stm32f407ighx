/**
 * @file driver.c
 * @brief DR16遥控器驱动实现文件
 *
 * 实现DR16遥控器的UART通信配置和控制消息解析功能
 */

#include "driver.h"
#include "Tasks/protect.h"
#include "main.h"
#include "usart.h"
#include <string.h>

static u8 uart_rx_buffer[DMA_BUFFER_SIZE];               ///< UART接收缓冲区
static volatile rcCtrl_dr16 rc_ctrl;                     ///< 遥控器控制信息
static volatile uint32_t last_dma_pos = DMA_BUFFER_SIZE; // 上次读取位置

void rc_dr16_setup(void) {
  // 启动循环 DMA 接收（关键：Circular Mode）
  if (HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, DMA_BUFFER_SIZE) !=
      HAL_OK) {
    Error_Handler();
  }
  // 使能 IDLE 中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  // 初始化 last_dma_pos
  last_dma_pos = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
}

volatile rcCtrl_dr16 *rc_dr16_get_ctrl_sig() { return &rc_ctrl; }

void USART3_IRQHandler(void) {
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) &&
      __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE)) {

    __HAL_UART_CLEAR_IDLEFLAG(&huart3); // 必须先清除标志

    uint32_t curr_dma_pos = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    uint32_t curr_pos =
        DMA_BUFFER_SIZE -
        curr_dma_pos; // 当前 DMA 写入位置（0 ~ DMA_BUFFER_SIZE-1）

    // 计算从 last_dma_pos 到 curr_pos 之间新收到的数据长度
    int32_t new_data_len;
    if (curr_pos >= last_dma_pos) {
      new_data_len = curr_pos - last_dma_pos;
    } else {
      new_data_len = DMA_BUFFER_SIZE - last_dma_pos + curr_pos;
    }

    // 如果有新数据，尝试从中提取完整 DR16 包（18 字节）
    if (new_data_len >= 18) {
      // 构造临时缓冲区，拼接环形数据（最多拷贝 18 字节）
      uint8_t temp_buf[18];
      uint32_t copy_start = last_dma_pos;
      if (copy_start + 18 <= DMA_BUFFER_SIZE) {
        // 未跨尾
        memcpy(temp_buf, &uart_rx_buffer[copy_start], 18);
      } else {
        // 跨尾：分两段拷贝
        uint32_t first_part = DMA_BUFFER_SIZE - copy_start;
        memcpy(temp_buf, &uart_rx_buffer[copy_start], first_part);
        memcpy(&temp_buf[first_part], uart_rx_buffer, 18 - first_part);
      }

      rc_ctrl_msg_parse_dr16(temp_buf, &rc_ctrl);
      protect_refresh_idle_time();

      // 更新 last_dma_pos：只前进 18 字节（假设每包固定 18 字节）
      last_dma_pos = (last_dma_pos + new_data_len) % DMA_BUFFER_SIZE;
    } else {
      // 数据不足，不处理（或可记录错误）
    }
  }
}