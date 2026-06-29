#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

LOG_MODULE_REGISTER(usb_float51_logger, LOG_LEVEL_INF);

#define CTRL_SYNC_ID 0x0303
#define CTRL_FLOAT_NUM 7
#define CTRL_BUTTON_NUM 48
#define CTRL_BUTTON_MASK_NUM 3
#define CTRL_STICK_LOG_DELTA 0.05f
#define CTRL_STICK_LOG_INTERVAL_MS 1000

enum ctrl_data_index {
	CTRL_IDX_BUTTONS_0_15 = 0,
	CTRL_IDX_BUTTONS_16_31,
	CTRL_IDX_BUTTONS_32_47,
	CTRL_IDX_LX,
	CTRL_IDX_LY,
	CTRL_IDX_RX,
	CTRL_IDX_RY,
};

DUAL_PROPOSE_PROTOCOL_DEFINE(control_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);

static float ctrl_rx[CTRL_FLOAT_NUM];
static float ctrl_data[CTRL_FLOAT_NUM];
static uint32_t ctrl_rx_count;

K_SEM_DEFINE(ctrl_rx_sem, 0, 1);

static const char *const ctrl_button_names[CTRL_BUTTON_NUM] = {
	[0] = "LEFT_TRACKPAD",
	[1] = "RIGHT_TRACKPAD",
	[2] = "QUICK_ACCESS",
	[3] = "A",
	[4] = "B",
	[5] = "X",
	[6] = "Y",
	[7] = "LB",
	[8] = "RB",
	[9] = "LT_FULL",
	[10] = "RT_FULL",
	[11] = "VIEW",
	[12] = "MENU",
	[13] = "STEAM",
	[14] = "L3",
	[15] = "R3",
	[16] = "DPAD_UP",
	[17] = "DPAD_DOWN",
	[18] = "DPAD_LEFT",
	[19] = "DPAD_RIGHT",
	[20] = "L4",
	[21] = "R4",
	[22] = "L5",
	[23] = "R5",
	[32] = "VIRTUAL_BUTTON_1",
	[33] = "VIRTUAL_BUTTON_2",
	[34] = "VIRTUAL_BUTTON_3",
	[35] = "VIRTUAL_BUTTON_4",
	[36] = "VIRTUAL_BUTTON_5",
	[37] = "VIRTUAL_BUTTON_6",
	[38] = "VIRTUAL_BUTTON_7",
	[39] = "VIRTUAL_BUTTON_8",
	[40] = "ACTION_SELECT_3_LEFT",
	[41] = "ACTION_SELECT_3_MID",
	[42] = "ACTION_SELECT_3_RIGHT",
	[43] = "ACTION_SELECT_2_LEFT",
	[44] = "ACTION_SELECT_2_MID",
	[45] = "ACTION_SELECT_2_RIGHT",
	[46] = "ACTION_RELEASE",
	[47] = "ACTION_PLACE",
};

static uint16_t ctrl_float_to_mask(float value)
{
	if (value <= 0.0f) {
		return 0U;
	}

	if (value >= 65535.0f) {
		return UINT16_MAX;
	}

	return (uint16_t)(value + 0.5f);
}

static bool ctrl_button_pressed(const uint16_t masks[CTRL_BUTTON_MASK_NUM], uint8_t button_id)
{
	return (masks[button_id / 16U] & BIT(button_id % 16U)) != 0U;
}

static bool stick_changed(float curr, float prev)
{
	float diff = curr - prev;

	if (diff < 0.0f) {
		diff = -diff;
	}

	return diff >= CTRL_STICK_LOG_DELTA;
}

static void control_rx_cb(int status)
{
	if (status != SYNC_PACK_STATUS_DONE) {
		return;
	}

	memcpy(ctrl_data, ctrl_rx, sizeof(ctrl_data));
	ctrl_rx_count++;
	k_sem_give(&ctrl_rx_sem);
}

int main(void)
{
	int ret;
	sync_table_t *control_rx;
	uint16_t prev_masks[CTRL_BUTTON_MASK_NUM] = {0};
	float prev_sticks[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	uint32_t last_stick_log_ms = 0U;
	bool have_prev = false;

	LOG_INF("usb_float51_logger start");

	ret = ares_bind_interface(&usb_bulk_interface, &control_protocol);
	if (ret != 0) {
		LOG_ERR("failed to initialize ARES USB interface: %d", ret);
		return ret;
	}

	control_rx = dual_sync_add(&control_protocol, CTRL_SYNC_ID, (uint8_t *)ctrl_rx,
				   sizeof(ctrl_rx), control_rx_cb);
	if (control_rx == NULL) {
		LOG_ERR("failed to register control sync pack");
		return -ENOMEM;
	}

	LOG_INF("waiting for sync id=0x%04x payload=%d floats", CTRL_SYNC_ID, CTRL_FLOAT_NUM);
	LOG_INF("button logs show PRESSED/RELEASED, stick logs show lx/ly/rx/ry");

	while (1) {
		uint16_t curr_masks[CTRL_BUTTON_MASK_NUM];
		bool any_button_change = false;
		bool any_stick_change = false;

		k_sem_take(&ctrl_rx_sem, K_FOREVER);

		curr_masks[0] = ctrl_float_to_mask(ctrl_data[CTRL_IDX_BUTTONS_0_15]);
		curr_masks[1] = ctrl_float_to_mask(ctrl_data[CTRL_IDX_BUTTONS_16_31]);
		curr_masks[2] = ctrl_float_to_mask(ctrl_data[CTRL_IDX_BUTTONS_32_47]);

		for (uint8_t button_id = 0U; button_id < CTRL_BUTTON_NUM; button_id++) {
			bool curr_pressed = ctrl_button_pressed(curr_masks, button_id);
			bool prev_pressed = ctrl_button_pressed(prev_masks, button_id);
			const char *name = ctrl_button_names[button_id];

			if (curr_pressed == prev_pressed) {
				continue;
			}

			any_button_change = true;
			LOG_INF("button[%u]=%s %s", button_id,
				name != NULL ? name : "UNKNOWN",
				curr_pressed ? "PRESSED" : "RELEASED");
		}

		if (!have_prev ||
		    stick_changed(ctrl_data[CTRL_IDX_LX], prev_sticks[0]) ||
		    stick_changed(ctrl_data[CTRL_IDX_LY], prev_sticks[1]) ||
		    stick_changed(ctrl_data[CTRL_IDX_RX], prev_sticks[2]) ||
		    stick_changed(ctrl_data[CTRL_IDX_RY], prev_sticks[3])) {
			uint32_t now = k_uptime_get_32();

			if (!have_prev || (now - last_stick_log_ms) >= CTRL_STICK_LOG_INTERVAL_MS) {
				any_stick_change = true;
				last_stick_log_ms = now;
				LOG_INF("sticks L(%.3f, %.3f) R(%.3f, %.3f) rx_count=%u",
					(double)ctrl_data[CTRL_IDX_LX], (double)ctrl_data[CTRL_IDX_LY],
					(double)ctrl_data[CTRL_IDX_RX], (double)ctrl_data[CTRL_IDX_RY],
					ctrl_rx_count);
			}
		}


		memcpy(prev_masks, curr_masks, sizeof(prev_masks));
		prev_sticks[0] = ctrl_data[CTRL_IDX_LX];
		prev_sticks[1] = ctrl_data[CTRL_IDX_LY];
		prev_sticks[2] = ctrl_data[CTRL_IDX_RX];
		prev_sticks[3] = ctrl_data[CTRL_IDX_RY];
		have_prev = true;
	}

	return 0;
}
