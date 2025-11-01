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

  // 配置 BMI088
  bmi088_setup(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, &htim10,
               TIM_CHANNEL_1, GPIO_PIN_4, GPIO_PIN_5);
  bsp_gpio_exti_cb_add(
      bmi088_update_pose); // 接收到 BMI088 数据准备完成中断后更新姿态

  bsp_cron_job_add(bmi088_temp_ctrl); // 定期陀螺仪温控

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
}