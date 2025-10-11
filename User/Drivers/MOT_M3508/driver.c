#include "driver.h"
#include "main.h"

void mot_setup_can_m3508(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef can_filter = {0}; // 初始化为0更安全

  // --- 过滤器组 0: ID 0x201, 0x202 ---
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (0x201U << 5); // ID 0x201
  can_filter.FilterIdLow = (0x202U << 5);  // ID 0x202
  can_filter.FilterMaskIdHigh = 0;         // 列表模式下不使用掩码
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank =
      14;

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // --- 过滤器组 1: ID 0x203, 0x204 ---
  can_filter.FilterBank = 1;
  can_filter.FilterIdHigh = (0x203U << 5);
  can_filter.FilterIdLow = (0x204U << 5);
  // 保持其他字段与上面相同（FIFO0, IDLIST等）

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // --- 过滤器组 2: ID 0x205, 0x206 ---
  can_filter.FilterBank = 2;
  can_filter.FilterIdHigh = (0x205U << 5);
  can_filter.FilterIdLow = (0x206U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // --- 过滤器组 3: ID 0x207, 0x208 ---
  can_filter.FilterBank = 3;
  can_filter.FilterIdHigh = (0x207U << 5);
  can_filter.FilterIdLow = (0x208U << 5);

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }

  // 启动CAN外设
  if (HAL_CAN_Start(hcan) != HAL_OK) {
    Error_Handler();
  }

  // 激活CAN RX中断仅用于FIFO0
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK) {
    Error_Handler();
  }
}