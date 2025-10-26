#include "protect.h"
#include "driver.h"

extern volatile u8 dmj4310_protect_on;
void protect_dmj4310() {
  dmj4310_protect_on = true;
  dmj4310_disable();
  dmj4310_reset_pos();
}
void lift_protect_dmj4310() { dmj4310_protect_on = false; }