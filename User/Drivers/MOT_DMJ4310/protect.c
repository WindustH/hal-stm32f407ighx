#include "protect.h"
#include "driver.h"

void mot_protect_dmj4310() {
  mot_set_torque_dmj4310(0.0f);
  mot_send_ctrl_msg_dmj4310();
  PROTECT_ON = true;
}