#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "control_packet.h"
#include "module_lift.h"
#include "module_r1_catch.h"
#include "module_sbus2.h"
#include "usb_control.h"

LOG_MODULE_REGISTER(team_usb_integrated, LOG_LEVEL_INF);

int main(void)
{
	int ret;
	struct team_usb_packet packet;

	memset(&packet, 0, sizeof(packet));

	LOG_INF("team usb integrated start");

	ret = usb_control_init();
	if (ret != 0) {
		return ret;
	}

	ret = module_r1_catch_init();
	if (ret != 0) {
		LOG_WRN("R1 catch module unavailable: %d", ret);
	}

	ret = module_lift_init();
	if (ret != 0) {
		LOG_WRN("lift module unavailable: %d", ret);
	}

	ret = module_sbus2_init();
	if (ret != 0) {
		LOG_WRN("sbus_2 module unavailable: %d", ret);
	}

	while (1) {
		bool packet_updated = usb_control_poll(&packet, K_MSEC(5));

		module_r1_catch_update(&packet, packet_updated);
		module_lift_update(&packet, packet_updated);
		module_sbus2_update(&packet, packet_updated);
	}

	return 0;
}
