#include "protect.h"
#include "driver.h"

extern u8 M3508_PROTECT_ON;
void mot_protect_m3508() {
  for (u8 i = 0; i < 8; i++)
    mot_set_current_m3508(i, 0);
  mot_send_ctrl_msg_m3508();
  M3508_PROTECT_ON = true;
}