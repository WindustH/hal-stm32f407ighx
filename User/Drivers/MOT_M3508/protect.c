#include "protect.h"
#include "driver.h"

extern volatile u8 m3508_protect_on;
void protect_m3508() {
  for (u8 i = 0; i < 8; i++)
    m3508_set_current(i, 0);
  m3508_send_ctrl_msg();
  m3508_protect_on = true;
}

void lift_protect_m3508() { m3508_protect_on = false; }