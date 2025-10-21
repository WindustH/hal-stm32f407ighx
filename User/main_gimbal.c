#ifdef BOARD_GIMBAL
#include "main_gimbal.h"
#include "BSP/all.h"     // IWYU pragma: keep
#include "Drivers/all.h" // IWYU pragma: keep
#include "Tasks/all.h"   // IWYU pragma: keep
#include "handle.h"
#include "main.h"
#include "type.h"

volatile f32 *m3508_pidv_feedback[8] = {NULL};
volatile f32 *m3508_pidx_feedback[8] = {NULL};
volatile f32 *dmj6006_pidv_feedback;
volatile f32 *dmj6006_pidx_feedback;

void main_gimbal() {
  // 配置 DMJ4310
  dmj4310_setup(&hcan1, 0x002U, 0x003U, 0, CAN_FILTER_FIFO0);
  bsp_can_fifo0_cb_add(dmj4310_update_stat); // 从 CAN FIFO 中断接收更新
  bsp_cron_job_add(dmj4310_send_ctrl_msg);   // 定期发送控制信号
  dmj4310_set_torque(0.0f);

  // 配置 DMJ6006
  dmj6006_setup(&hcan1, 0x001U, 0x000U, 1, CAN_FILTER_FIFO1);
  bsp_can_fifo1_cb_add(dmj6006_update_stat);
  bsp_cron_job_add(dmj6006_send_ctrl_msg);
  dmj6006_set_torque(0.0f);

  // 配置 M3508
  m3508_setup(&hcan2, 14, CAN_FILTER_FIFO0);
  bsp_can_fifo0_cb_add(m3508_update_stat);
  bsp_cron_job_add(m3508_send_ctrl_msg);

  // 配置 RC DR16 和 板间通讯发送
  bc_gim_tx_setup(&hcan1, 0x007U);
  rc_dr16_setup();
  dr16_cb_add(bc_gim_send_msg);                  // 接收到遥控器信号转发给底盘
  dr16_cb_add(gimbal_protect_refresh_idle_time); // 接收到信号清零信号空闲时间
  // 配置板间通讯接收
  // bc_gim_rx_setup(&hcan1, 0x008U, 2, CAN_FILTER_FIFO0);
  // bsp_can_fifo0_cb_add(bc_gim_update_data_from_cha);
  // 配置 BMI088
  bmi088_setup(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, &htim10,
               TIM_CHANNEL_1, GPIO_PIN_4, GPIO_PIN_5);

  bsp_gpio_exti_cb_add(
      bmi088_update_pose); // 接收到 BMI088 数据准备完成中断后更新姿态
  bsp_cron_job_add(bmi088_temp_ctrl); // 定期陀螺仪温控

  start_hal_peripherals();
  // 外设启动后 --------------------------------------------

  bsp_cron_job_add(gimbal_protect_update_idle_time); // 定期更新信号空闲时间
  gimbal_protect_start();                            // 启用云台保护
  // 启动 BMI088
  bmi088_start();

  for (u8 i = 0; i < 8; i++) {
    m3508_pidv_feedback[i] = &m3508_get_stat(i)->v;
    m3508_pidx_feedback[i] = &m3508_get_stat(i)->x;
  } // 设置 PID 反馈源

  m3508_pidv_setup(m3508_pidv_feedback);
  m3508_pidx_setup(m3508_pidx_feedback);
  bsp_cron_job_add(m3508_pidv_update);
  bsp_cron_job_add(m3508_pidx_update);
  m3508_pidv_start();
  m3508_pidx_start();
  m3508_pidx_set_target(0, 0.0f);

  dmj6006_pidv_feedback = &dmj6006_get_stat()->v;
  dmj6006_pidv_setup(dmj6006_pidv_feedback);
  bsp_cron_job_add(dmj6006_pidv_update);
  dmj6006_pidv_start();

  dmj6006_pidx_feedback = &bmi088_get_pose()->yaw;
  dmj6006_pidx_setup(dmj6006_pidx_feedback);
  bsp_cron_job_add(dmj6006_pidx_update);
  dmj6006_pidx_start();
}
#endif
