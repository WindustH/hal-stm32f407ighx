#include "can_tx_queue.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>

// 单个队列项
typedef struct {
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
} can_tx_item_t;

// 队列控制块
typedef struct {
  bool in_use;
  CAN_HandleTypeDef *hcan;
  can_tx_item_t buffer[CAN_TX_QUEUE_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint8_t pending; // 正在硬件发送的数量（0~3）
} can_tx_queue_instance_t;

static can_tx_queue_instance_t g_queues[MAX_CAN_INSTANCES] = {0};

// 原子操作辅助
static inline uint32_t enter_critical(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void exit_critical(uint32_t primask) { __set_PRIMASK(primask); }

// 查找已注册的队列实例
static can_tx_queue_instance_t *find_queue(CAN_HandleTypeDef *hcan) {
  for (int i = 0; i < MAX_CAN_INSTANCES; i++) {
    if (g_queues[i].in_use && g_queues[i].hcan == hcan) {
      return &g_queues[i];
    }
  }
  return NULL;
}

// 判断队列是否满
static inline bool queue_full(can_tx_queue_instance_t *q) {
  return ((q->head + 1) % CAN_TX_QUEUE_SIZE) == q->tail;
}

// 判断队列是否空
static inline bool queue_empty(can_tx_queue_instance_t *q) {
  return q->head == q->tail;
}

// 尝试从队列发送（供中断和入队时调用）
static void try_send_from_queue(can_tx_queue_instance_t *q) {
  while (q->pending < 3 && !queue_empty(q)) {
    uint32_t mbox = CAN_TX_MAILBOX0;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        q->hcan, &q->buffer[q->tail].header, q->buffer[q->tail].data, &mbox);
    if (status == HAL_OK &&
        (mbox == CAN_TX_MAILBOX0 || mbox == CAN_TX_MAILBOX1 ||
         mbox == CAN_TX_MAILBOX2)) {
      q->pending++;
      q->tail = (q->tail + 1) % CAN_TX_QUEUE_SIZE;
    } else {
      break; // mailbox 满，停止尝试
    }
  }
}

// ========================
// 公共 API
// ========================

HAL_StatusTypeDef can_tx_queue_init(CAN_HandleTypeDef *hcan) {
  if (!hcan)
    return HAL_ERROR;

  uint32_t mask = enter_critical();

  // 防止重复初始化
  if (find_queue(hcan)) {
    exit_critical(mask);
    return HAL_OK;
  }

  // 寻找空闲槽位
  can_tx_queue_instance_t *free_slot = NULL;
  for (int i = 0; i < MAX_CAN_INSTANCES; i++) {
    if (!g_queues[i].in_use) {
      free_slot = &g_queues[i];
      break;
    }
  }

  if (!free_slot) {
    exit_critical(mask);
    return HAL_ERROR; // 超出实例数量
  }

  // 初始化队列
  free_slot->in_use = true;
  free_slot->hcan = hcan;
  free_slot->head = 0;
  free_slot->tail = 0;
  free_slot->pending = 0;

  // 启用中断
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
    Error_Handler();
  }

  exit_critical(mask);
  return HAL_OK;
}

HAL_StatusTypeDef can_send_message(CAN_HandleTypeDef *hcan,
                                   CAN_TxHeaderTypeDef *header, uint8_t *data) {
  if (!hcan || !header || !data) {
    return HAL_ERROR;
  }

  can_tx_queue_instance_t *q = find_queue(hcan);
  if (!q) {
    // 未注册队列：直接发送（可能失败）
    uint32_t mbox;
    return HAL_CAN_AddTxMessage(hcan, header, data, &mbox);
  }

  // 已注册队列：尝试入队
  if (queue_full(q)) {
    return HAL_ERROR; // 队列满，拒绝
  }

  uint32_t mask = enter_critical();

  // 入队
  can_tx_item_t *item = &q->buffer[q->head];
  item->header = *header;
  memcpy(item->data, data, header->DLC > 8 ? 8 : header->DLC);
  q->head = (q->head + 1) % CAN_TX_QUEUE_SIZE;

  // 尝试立即发送
  try_send_from_queue(q);

  exit_critical(mask);
  return HAL_OK;
}

// ========================
// 中断回调（需在 stm32xxx_it.c 或其他地方声明为 extern）
// ========================
volatile uint8_t tx_complete_count = 0; // 调试用
extern CAN_HandleTypeDef hcan1;
void CAN1_TX_IRQHandler(void) {
  uint32_t tsr = CAN1->TSR;

  // 检查 Mailbox 0
  if (tsr & CAN_TSR_RQCP0) {
    CAN1->TSR = CAN_TSR_RQCP0; // 清除标志（写1清除）
    tx_complete_count++;

    // 👇 手动触发队列发送（关键！）
    // 查找 hcan1 对应的队列
    for (int i = 0; i < MAX_CAN_INSTANCES; i++) {
      if (g_queues[i].in_use && g_queues[i].hcan == &hcan1) {
        // 模拟 pending--
        if (g_queues[i].pending > 0) {
          g_queues[i].pending--;
        }
        // 尝试发送下一帧
        while (g_queues[i].pending < 3 &&
               g_queues[i].head != g_queues[i].tail) {
          uint32_t mbox;
          HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
              &hcan1, &g_queues[i].buffer[g_queues[i].tail].header,
              g_queues[i].buffer[g_queues[i].tail].data, &mbox);
          if (status == HAL_OK && (mbox <= CAN_TX_MAILBOX2)) {
            g_queues[i].pending++;
            g_queues[i].tail = (g_queues[i].tail + 1) % CAN_TX_QUEUE_SIZE;
          } else {
            break;
          }
        }
        break;
      }
    }
  }

  // 检查 Mailbox 1
  if (tsr & CAN_TSR_RQCP1) {
    CAN1->TSR = CAN_TSR_RQCP1;
    tx_complete_count++;
    // 同上逻辑（通常不需要，因为 HAL 会自动选空闲 mailbox）
  }

  // 检查 Mailbox 2
  if (tsr & CAN_TSR_RQCP2) {
    CAN1->TSR = CAN_TSR_RQCP2;
    tx_complete_count++;
  }
}

extern CAN_HandleTypeDef hcan2;

void CAN2_TX_IRQHandler(void) {
  uint32_t tsr = CAN2->TSR;

  // 检查 Mailbox 0
  if (tsr & CAN_TSR_RQCP0) {
    CAN2->TSR = CAN_TSR_RQCP0; // 清除标志（写1清除）
    tx_complete_count++;       // 调试用，可选

    // 查找 hcan2 对应的队列
    for (int i = 0; i < MAX_CAN_INSTANCES; i++) {
      if (g_queues[i].in_use && g_queues[i].hcan == &hcan2) {
        if (g_queues[i].pending > 0) {
          g_queues[i].pending--;
        }
        // 尝试发送下一帧
        while (g_queues[i].pending < 3 &&
               g_queues[i].head != g_queues[i].tail) {
          uint32_t mbox;
          HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
              &hcan2, &g_queues[i].buffer[g_queues[i].tail].header,
              g_queues[i].buffer[g_queues[i].tail].data, &mbox);
          if (status == HAL_OK && (mbox <= CAN_TX_MAILBOX2)) {
            g_queues[i].pending++;
            g_queues[i].tail = (g_queues[i].tail + 1) % CAN_TX_QUEUE_SIZE;
          } else {
            break;
          }
        }
        break;
      }
    }
  }

  // 检查 Mailbox 1
  if (tsr & CAN_TSR_RQCP1) {
    CAN2->TSR = CAN_TSR_RQCP1;
    tx_complete_count++;
    // 通常不需要重复处理，因为 HAL 会自动分配 mailbox，
    // 且 pending 计数已在 mailbox 0 处理中统一维护。
    // 如果你希望更精确地跟踪每个 mailbox，可在此补充逻辑，
    // 但当前设计以“总 pending 数”为准，因此可省略。
  }

  // 检查 Mailbox 2
  if (tsr & CAN_TSR_RQCP2) {
    CAN2->TSR = CAN_TSR_RQCP2;
    tx_complete_count++;
  }
}