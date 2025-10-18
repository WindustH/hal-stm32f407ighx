#ifndef __USER_BSP_CAN_TX_QUEUE__
#define __USER_BSP_CAN_TX_QUEUE__

#define CAN_TX_QUEUE_SIZE 32 // 每个 CAN 实例的队列深度
#define MAX_CAN_INSTANCES 2  // 支持的 CAN 控制器最大数量（CAN1/CAN2）
#include "stm32f4xx_hal.h"   // IWYU pragma: keep

// 初始化一个 CAN 发送队列
HAL_StatusTypeDef can_tx_queue_init(CAN_HandleTypeDef *hcan);

// 统一发送接口：自动选择队列 or 直发
HAL_StatusTypeDef can_send_message(CAN_HandleTypeDef *hcan,
                                   CAN_TxHeaderTypeDef *header, uint8_t *data);
#endif /* __USER_BSP_CAN_TX_QUEUE__ */
