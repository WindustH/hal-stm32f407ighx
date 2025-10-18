#include "main_gimbal.h"
#include "BSP/all.h"     // IWYU pragma: keep
#include "Drivers/all.h" // IWYU pragma: keep
#include "Tasks/all.h"   // IWYU pragma: keep
#include "handle.h"
#include "main.h"

extern volatile f32 *m3508_pidv_feedback[8];
extern volatile f32 *m3508_pidx_feedback[8];
volatile f32 *dmj6006_pidv_feedback;

void main_gimbal() {
  // 配置 DMJ4310
  dmj4310_setup(&hcan1, 0x002U, 0x003U, 0);
  bsp_can_fifo0_cb_add(dmj4310_update_stat);
  bsp_cron_job_add(dmj4310_send_ctrl_msg);
  dmj4310_set_torque(0.0f);

  // 配置 DMJ6006
  dmj6006_setup(&hcan1, 0x001U, 0x000U, 1);
  bsp_can_fifo0_cb_add(dmj6006_update_stat);
  bsp_cron_job_add(dmj6006_send_ctrl_msg);
  dmj6006_set_torque(0.0f);

  // 配置 M3508
  m3508_setup(&hcan2, 14);
  bsp_can_fifo0_cb_add(m3508_update_stat);
  bsp_cron_job_add(m3508_send_ctrl_msg);

  // 配置 RC DR16
  board_com_tx_setup(&hcan1, 0x007U);
  rc_dr16_setup();

  // 配置 BMI088
  bmi088_setup(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, &htim10,
               TIM_CHANNEL_1, GPIO_PIN_4, GPIO_PIN_5);

  bsp_gpio_exti_cb_add(bmi088_update_pose);
  bsp_cron_job_add(bmi088_temp_ctrl);
  // 外设启动
  start_hal_peripherals();

  bsp_cron_job_add(gimbal_protect_update_idle_time);
  gimbal_protect_start();
  // 启动陀螺仪
  bmi088_start();

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

  volatile f32 *dmj6006_pidv_feedback = &dmj6006_get_stat()->v;
  dmj6006_pidv_setup(dmj6006_pidv_feedback);
  dmj6006_pidv_start();
}