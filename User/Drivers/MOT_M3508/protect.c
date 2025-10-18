#include "protect.h"
#include "driver.h"

extern volatile u8 M3508_PROTECT_ON;
void protect_m3508() {
  for (u8 i = 0; i < 8; i++)
    m3508_set_current(i, 0);
  m3508_send_ctrl_msg();
  M3508_PROTECT_ON = true;
}

void lift_protect_m3508() { M3508_PROTECT_ON = false; }