#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "module_lift.h"

LOG_MODULE_REGISTER(team_usb_lift, LOG_LEVEL_INF);

#define CTRL_BUTTON_A_ID 3U
#define CTRL_BUTTON_Y_ID 6U

#define DMOTOR_NODE1 DT_NODELABEL(dm_motor000)
#define DMOTOR_NODE2 DT_NODELABEL(dm_motor001)
#define DMOTOR_NODE3 DT_NODELABEL(dm_motor002)
#define DMOTOR_NODE4 DT_NODELABEL(dm_motor003)

static const struct device *dm_motor[4] = {
	DEVICE_DT_GET(DMOTOR_NODE1),
	DEVICE_DT_GET(DMOTOR_NODE2),
	DEVICE_DT_GET(DMOTOR_NODE3),
	DEVICE_DT_GET(DMOTOR_NODE4),
};

static float action[4];
static const int rank_map[4] = {-1, -1, 1, 1};
static const float motor_ff[4] = {0.5f, 1.8f, 0.5f, 1.8f};
static const float gear_angle_ac[] = {0.0f, 360.0f, 720.0f, 1440.0f, 3600.0f, 4800.0f};
static const float gear_angle_bd[] = {0.0f, 410.0f, 770.0f, 1490.0f, 3650.0f, 4850.0f};

static int current_gear;
static bool last_a_pressed;
static bool last_y_pressed;
static bool module_ready;

static void lifthigh(int id, float target_deg)
{
	float torque_ff = 0.0f;

	if (!device_is_ready(dm_motor[id])) {
		return;
	}

	if (target_deg > 0.0f) {
		torque_ff = motor_ff[id];
	} else if (target_deg < 0.0f) {
		torque_ff = -motor_ff[id];
	}
	// LOG_INF("current_gear: %d", current_gear);
	motor_set_mit(dm_motor[id], 0.0f, target_deg, torque_ff);
}

static void hold_current_target(void)
{
	for (int i = 0; i < 4; i++) {
		lifthigh(i, action[i]);
	}
}

static void apply_current_gear(void)
{
	float target_angle_ac = gear_angle_ac[current_gear];
	float target_angle_bd = gear_angle_bd[current_gear];

	action[0] = target_angle_ac * rank_map[0];
	action[1] = target_angle_bd * rank_map[1];
	action[2] = target_angle_ac * rank_map[2];
	action[3] = target_angle_bd * rank_map[3];
	LOG_INF("lift gear=%d/%d", current_gear + 1, ARRAY_SIZE(gear_angle_ac));
}

static void gear_step(int delta)
{
	int prev_gear = current_gear;

	if (delta > 0 && current_gear < (int)ARRAY_SIZE(gear_angle_ac) - 1) {
		current_gear++;
	} else if (delta < 0 && current_gear > 0) {
		current_gear--;
	}

	if (current_gear == prev_gear) {
		return;
	}

	apply_current_gear();
}

int module_lift_init(void)
{
	for (int i = 0; i < 4; i++) {
		if (!device_is_ready(dm_motor[i])) {
			LOG_WRN("lift motor %d not ready, module disabled", i);
			return -ENODEV;
		}

		motor_control(dm_motor[i], ENABLE_MOTOR);
		k_msleep(10);
		motor_set_mode(dm_motor[i], MIT);
		k_msleep(10);
	}

	apply_current_gear();
	module_ready = true;
	LOG_INF("lift module ready");
	return 0;
}

void module_lift_update(const struct team_usb_packet *packet, bool packet_updated)
{
	bool a_pressed;
	bool y_pressed;

	if (!module_ready) {
		return;
	}

	if (packet_updated) {
		a_pressed = team_usb_button_pressed(packet, CTRL_BUTTON_A_ID);
		y_pressed = team_usb_button_pressed(packet, CTRL_BUTTON_Y_ID);

		if (y_pressed && !last_y_pressed) {
			gear_step(1);
		}

		if (a_pressed && !last_a_pressed) {
			gear_step(-1);
		}

		last_a_pressed = a_pressed;
		last_y_pressed = y_pressed;
	}

	hold_current_target();
}
