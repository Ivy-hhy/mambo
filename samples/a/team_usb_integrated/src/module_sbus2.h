#ifndef TEAM_USB_MODULE_SBUS2_H_
#define TEAM_USB_MODULE_SBUS2_H_

#include <stdbool.h>

#include "control_packet.h"

int module_sbus2_init(void);
void module_sbus2_update(const struct team_usb_packet *packet, bool packet_updated);

#endif
