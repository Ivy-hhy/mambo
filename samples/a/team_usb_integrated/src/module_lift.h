#ifndef TEAM_USB_MODULE_LIFT_H_
#define TEAM_USB_MODULE_LIFT_H_

#include <stdbool.h>

#include "control_packet.h"

int module_lift_init(void);
void module_lift_update(const struct team_usb_packet *packet, bool packet_updated);

#endif
