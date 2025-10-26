#include "protect.h"
#include "driver.h"

extern volatile u8 dmj6006_protect_on;
void protect_dmj6006() {
  dmj6006_protect_on = true;
  dmj6006_disable();
  dmj6006_reset_pos();
}
void lift_protect_dmj6006() { dmj6006_protect_on = false; }