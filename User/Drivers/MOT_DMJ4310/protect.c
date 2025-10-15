#include "protect.h"
#include "driver.h"

extern u8 DMJ4310_PROTECT_ON;
void mot_protect_dmj4310() {
  dmj4310_set_torque(0.0f);
  dmj4310_send_ctrl_msg();
  DMJ4310_PROTECT_ON = true;
}