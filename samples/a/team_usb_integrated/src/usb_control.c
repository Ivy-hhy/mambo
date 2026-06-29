#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

#include "usb_control.h"

LOG_MODULE_REGISTER(team_usb_comm, LOG_LEVEL_INF);

DUAL_PROPOSE_PROTOCOL_DEFINE(control_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static float ctrl_rx[TEAM_USB_FLOAT_NUM];
static float ctrl_data[TEAM_USB_FLOAT_NUM];
static uint32_t ctrl_rx_count;
static sync_table_t *control_rx;

K_SEM_DEFINE(ctrl_rx_sem, 0, 1);

static void control_rx_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	memcpy(ctrl_data, ctrl_rx, sizeof(ctrl_data));
	ctrl_rx_count++;
	k_sem_give(&ctrl_rx_sem);
}

int usb_control_init(void)
{
	int ret;

	ret = ares_bind_interface(&usb_bulk_interface, &control_protocol);
	if (ret != 0) {
		LOG_ERR("failed to initialize ARES USB interface: %d", ret);
		return ret;
	}

	control_rx = dual_sync_add(&control_protocol, TEAM_USB_SYNC_ID, (uint8_t *)ctrl_rx,
				   sizeof(ctrl_rx), control_rx_cb);
	if (control_rx == NULL) {
		LOG_ERR("failed to register control sync pack");
		return -ENOMEM;
	}

	LOG_INF("USB control ready: sync id=0x%04x payload=%d floats",
		TEAM_USB_SYNC_ID, TEAM_USB_FLOAT_NUM);
	return 0;
}

bool usb_control_poll(struct team_usb_packet *packet, k_timeout_t timeout)
{
	if (k_sem_take(&ctrl_rx_sem, timeout) != 0) {
		return false;
	}

	memcpy(packet->raw, ctrl_data, sizeof(packet->raw));
	packet->button_masks[0] = team_usb_float_to_mask(packet->raw[TEAM_USB_IDX_BUTTONS_0_15]);
	packet->button_masks[1] = team_usb_float_to_mask(packet->raw[TEAM_USB_IDX_BUTTONS_16_31]);
	packet->button_masks[2] = team_usb_float_to_mask(packet->raw[TEAM_USB_IDX_BUTTONS_32_47]);
	packet->rx_count = ctrl_rx_count;
	LOG_INF("b47=%d", team_usb_button_pressed(packet, 47));

	return true;
}
