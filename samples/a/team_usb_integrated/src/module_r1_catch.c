#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "module_r1_catch.h"

LOG_MODULE_REGISTER(team_usb_r1_catch, LOG_LEVEL_INF);

#define CTRL_BTN_CATCH_PREPARE 40U
#define CTRL_BTN_CATCH_RAISE 41U
#define CTRL_BTN_CATCH_ATTACK 42U
#define CTRL_BTN_CATCH_RELEASE 46U
#define CTRL_BTN_CATCH_SEIZE 47U

#define CATCH_ARM_NODE DT_NODELABEL(motor1)
#define CATCH_DM_NODE DT_NODELABEL(catch_dm_motor)
#define CATCH_VALVE_1_NODE DT_NODELABEL(catch_emvalve1)
#define CATCH_VALVE_2_NODE DT_NODELABEL(catch_emvalve2)

static const struct device *catch_arm_motor = DEVICE_DT_GET(CATCH_ARM_NODE);
static const struct device *catch_dm_motor = DEVICE_DT_GET(CATCH_DM_NODE);
static const struct gpio_dt_spec catch_valve1 = GPIO_DT_SPEC_GET(CATCH_VALVE_1_NODE, gpios);
static const struct gpio_dt_spec catch_valve2 = GPIO_DT_SPEC_GET(CATCH_VALVE_2_NODE, gpios);

static bool seize_prev;
static bool release_prev;
static bool prepare_prev;
static bool raise_prev;
static bool attack_prev;
static bool module_ready;

static void seize_action(void)
{
	gpio_pin_set_dt(&catch_valve1, true);
	k_msleep(500);
	LOG_INF("catch seize");
}

static void release_action(void)
{
	gpio_pin_set_dt(&catch_valve1, false);
	k_msleep(500);
	LOG_INF("catch release");
}

static void prepare_action(void)
{
	motor_set_angle(catch_arm_motor, 45.0f);
	k_msleep(100);
	motor_set_angle(catch_arm_motor, 90.0f);
	k_msleep(600);
	gpio_pin_set_dt(&catch_valve2, true);
	k_msleep(500);
	motor_set_mit(catch_dm_motor, 0.0f, 0.0f, 0.0f);
	LOG_INF("catch prepare");
}

static void raise_action(void)
{
	motor_set_mit(catch_dm_motor, 0.0f, 90.0f, 0.0f);
	LOG_INF("catch raise");
}

static void attack_action(void)
{
	gpio_pin_set_dt(&catch_valve2, false);
	k_msleep(300);
	motor_set_angle(catch_arm_motor, 179.5f);
	LOG_INF("catch attack");
}

int module_r1_catch_init(void)
{
	if (!device_is_ready(catch_arm_motor) || !device_is_ready(catch_dm_motor)) {
		LOG_WRN("R1 catch motors not ready, module disabled");
		return -ENODEV;
	}

	if (!device_is_ready(catch_valve1.port) || !device_is_ready(catch_valve2.port)) {
		LOG_WRN("R1 catch valves not ready, module disabled");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&catch_valve1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&catch_valve2, GPIO_OUTPUT_INACTIVE);

	motor_control(catch_arm_motor, SET_ZERO);
	motor_control(catch_dm_motor, SET_ZERO);
	k_msleep(500);
	motor_control(catch_dm_motor, ENABLE_MOTOR);
	motor_set_mode(catch_dm_motor, MIT);

	module_ready = true;
	LOG_INF("R1 catch module ready");
	return 0;
}

void module_r1_catch_update(const struct team_usb_packet *packet, bool packet_updated)
{
	bool seize_pressed;
	bool release_pressed;
	bool prepare_pressed;
	bool raise_pressed;
	bool attack_pressed;

	if (!module_ready) {
		return;
	}

	if (!packet_updated) {
		return;
	}

	seize_pressed = team_usb_button_pressed(packet, CTRL_BTN_CATCH_SEIZE);
	release_pressed = team_usb_button_pressed(packet, CTRL_BTN_CATCH_RELEASE);
	prepare_pressed = team_usb_button_pressed(packet, CTRL_BTN_CATCH_PREPARE);
	raise_pressed = team_usb_button_pressed(packet, CTRL_BTN_CATCH_RAISE);
	attack_pressed = team_usb_button_pressed(packet, CTRL_BTN_CATCH_ATTACK);

	if (release_pressed && !release_prev) {
		release_action();
	}

	if (seize_pressed && !seize_prev) {
		seize_action();
	}

	if (prepare_pressed && !prepare_prev) {
		prepare_action();
	}

	if (raise_pressed && !raise_prev) {
		raise_action();
	}

	if (attack_pressed && !attack_prev) {
		attack_action();
	}

	seize_prev = seize_pressed;
	release_prev = release_pressed;
	prepare_prev = prepare_pressed;
	raise_prev = raise_pressed;
	attack_prev = attack_pressed;
}
