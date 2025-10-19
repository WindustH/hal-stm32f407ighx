#ifdef BOARD_CHASSIS
#include "main_chassis.h"
#include "BSP/all.h"     // IWYU pragma: keep
#include "Drivers/all.h" // IWYU pragma: keep
#include "Tasks/all.h"   // IWYU pragma: keep
#include "handle.h"
#include "main.h"
#include "type.h"

volatile f32 *m3508_pidv_feedback[8] = {NULL};
volatile f32 *m3508_pidx_feedback[8] = {NULL};

void main_chassis() {
  // 配置 M3508
  m3508_setup(&hcan1, 0, CAN_FILTER_FIFO0);
  bsp_can_fifo0_cb_add(m3508_update_stat); // 从 CAN FIFO 中断接收更新
  bsp_cron_job_add(m3508_send_ctrl_msg);   // 定期发送控制信号
  // 配置板间通讯接收
  board_chassis_rx_setup(&hcan2, 0x007U, 14, CAN_FILTER_FIFO0);
  bsp_can_fifo0_cb_add(
      board_com_update_gimbal_data);             // 从 CAN FIFO 中断接收消息
  bcc_cb_add(chassis_update_wheel_speed);        // 根据接收的数据更新轮子速度
  bcc_cb_add(chassis_protect_refresh_idle_time); // 接收到消息清零空闲时间
  start_hal_peripherals();
  // 外设启动后 --------------------------------------------

  for (u8 i = 0; i < 8; i++) {
    m3508_pidv_feedback[i] = &m3508_get_stat(i)->v;
    m3508_pidx_feedback[i] = &m3508_get_stat(i)->x;
  } // 设置 PID 反馈源

  m3508_pidv_setup(m3508_pidv_feedback);
  // m3508_pidx_setup(m3508_pidx_feedback);
  bsp_cron_job_add(m3508_pidv_update);
  bsp_cron_job_add(m3508_pidx_update);
  m3508_pidv_start();
  // m3508_pidx_start();
  // m3508_pidx_set_target(0, 0.0f);
  // m3508_pidx_set_target(1, 0.0f);
  // m3508_pidx_set_target(2, 0.0f);
  // m3508_pidx_set_target(3, 0.0f);

  bsp_cron_job_add(chassis_protect_update_idle_time); // 定期刷新空闲时间
  chassis_protect_start();                            // 启用底盘保护
}
#endif