#include "protect.h"
#include "driver.h"

extern volatile u8 dmj6006_protect_on;
void protect_dmj6006() {
  dmj6006_set_torque(0.0f);
  dmj6006_send_ctrl_msg();
  dmj6006_protect_on = true;
}
void lift_protect_dmj6006() { dmj6006_protect_on = false; }