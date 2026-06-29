#ifndef TEAM_USB_MODULE_R1_CATCH_H_
#define TEAM_USB_MODULE_R1_CATCH_H_

#include <stdbool.h>

#include "control_packet.h"

int module_r1_catch_init(void);
void module_r1_catch_update(const struct team_usb_packet *packet, bool packet_updated);

#endif
