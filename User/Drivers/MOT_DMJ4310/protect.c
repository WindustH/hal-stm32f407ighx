#include "protect.h"
#include "driver.h"

extern u8 DMJ4310_PROTECT_ON;
void mot_protect_dmj4310() {
  mot_set_torque_dmj4310(0.0f);
  mot_send_ctrl_msg_dmj4310();
  DMJ4310_PROTECT_ON = true;
}