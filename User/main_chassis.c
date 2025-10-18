#include "main_chassis.h"
#ifdef BOARD_CHASSIS
#include "BSP/all.h"     // IWYU pragma: keep
#include "Drivers/all.h" // IWYU pragma: keep
#include "Tasks/all.h"   // IWYU pragma: keep
#include "handle.h"
#include "main.h"

volatile f32 *m3508_pidv_feedback[8] = {NULL};
volatile f32 *m3508_pidx_feedback[8] = {NULL};

void main_chassis() {
  // 配置 M3508
  m3508_setup(&hcan1, 0);
  bsp_can_fifo0_cb_add(m3508_update_stat);
  bsp_cron_job_add(m3508_send_ctrl_msg);

  board_com_rx_setup(&hcan2, 0x007U, 14);
  bsp_can_fifo0_cb_add(board_com_update_rx_data);
  // 外设启动
  start_hal_peripherals();

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
#endif