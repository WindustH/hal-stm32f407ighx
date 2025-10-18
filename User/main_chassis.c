#include "BSP/can_fifo0.h"
#include "BSP/can_tx_queue.h"
#include "BSP/cron.h"
#include "BSP/dwt.h"
#include "BSP/gpio_exti.h"
#include "Drivers/BMI880/driver.h"
#include "Drivers/BOARD_CAN_COM/driver.h"
#include "Drivers/MOT_DMJ4310/driver.h"
#include "Drivers/MOT_DMJ6006/driver.h"
#include "Drivers/MOT_M3508/driver.h"
#include "Drivers/RC_DR16/driver.h"
#include "Tasks/PID_DMJ4310/pidv.h"
#include "Tasks/PID_DMJ4310/pidx.h"
#include "Tasks/PID_M3508/pidv.h"
#include "Tasks/PID_M3508/pidx.h"
#include "Tasks/protect_chassis.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "main_gimbal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

extern volatile f32 *m3508_pidv_feedback[8];
extern volatile f32 *m3508_pidx_feedback[8];

void main_chassis() {
  // 配置 M3508
  m3508_setup(&hcan1, IS_MASTER_CAN, 0);
  bsp_can_fifo0_cb_add(m3508_update_stat);
  bsp_cron_job_add(m3508_send_ctrl_msg);

  board_com_rx_setup(&hcan2, IS_SLAVE_CAN, 0x007U, 14);
  bsp_can_fifo0_cb_add(board_com_update_rx_data);
  // 外设启动
  start_hal_peripherals();
  can_tx_manager_init(&hcan1);

  for (u8 i = 0; i < 8; i++) {
    m3508_pidv_feedback[i] = &m3508_get_stat(i)->v;
    m3508_pidx_feedback[i] = &m3508_get_stat(i)->x;
  }

  m3508_pidv_setup(m3508_pidv_feedback);
  m3508_pidx_setup(m3508_pidx_feedback);
  bsp_cron_job_add(m3508_pidv_update);
  bsp_cron_job_add(m3508_pidx_update);
  m3508_pidv_start();
  m3508_pidx_start();
  m3508_pidx_set_target(0, 0.0f);
  m3508_pidx_set_target(1, 0.0f);
  m3508_pidx_set_target(2, 0.0f);
  m3508_pidx_set_target(3, 0.0f);

  bsp_cron_job_add(chassis_protect_update_idle_time);
  chassis_protect_start();
}