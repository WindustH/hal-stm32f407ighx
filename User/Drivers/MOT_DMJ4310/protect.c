#include "protect.h"
#include "driver.h"

extern volatile u8 dmj4310_protect_on;
void protect_dmj4310() {
  dmj4310_set_torque(0.0f);
  dmj4310_send_ctrl_msg();
  dmj4310_protect_on = true;
}
void lift_protect_dmj4310() { dmj4310_protect_on = false; }