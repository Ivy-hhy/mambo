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

/* Button IDs (bits in the USB packet) */
#define CTRL_BTN_SWITCH  43U
#define CTRL_BTN_PREPARE 40U
#define CTRL_BTN_RAISE   41U
#define CTRL_BTN_ATTACK  42U

/* Posture states — flow tracking / logging only */
enum posture_state {
	POSTURE_P1,	/* idle:   arm=0°,    DM=0°,   valve2=off */
	POSTURE_P2,	/* ready4pluggin done:  arm=90°, DM=0°,   valve2=on  */
	POSTURE_P3,	/* raise:  arm=90°,   DM=90°,  valve2=on  */
	POSTURE_P4,	/* attack: arm=180.0°, DM=90.0°, valve2=off */
};

#define CATCH_ARM_NODE    DT_NODELABEL(motor1)
#define CATCH_DM_NODE     DT_NODELABEL(catch_dm_motor)
#define CATCH_VALVE_1_NODE DT_NODELABEL(catch_emvalve1)
#define CATCH_VALVE_2_NODE DT_NODELABEL(catch_emvalve2)

static const struct device    *catch_arm_motor = DEVICE_DT_GET(CATCH_ARM_NODE);
static const struct device    *catch_dm_motor  = DEVICE_DT_GET(CATCH_DM_NODE);
static const struct gpio_dt_spec catch_valve1  = GPIO_DT_SPEC_GET(CATCH_VALVE_1_NODE, gpios);
static const struct gpio_dt_spec catch_valve2  = GPIO_DT_SPEC_GET(CATCH_VALVE_2_NODE, gpios);

/* ---- MCU-persisted state (target values, not posture-enum-indexed) ---- */
static enum posture_state current_posture = POSTURE_P1;
static float   arm_target    = 0.0f;
static float   dm_target     = 0.0f;
static bool    valve2_target = false;
static bool    valve1_target = false; /* controlled by Button 43 (Switch) */

/* ---- Edge-detection history (Prepare / Raise / Attack only) ---- */
static bool prepare_prev;
static bool raise_prev;
static bool attack_prev;
static bool module_ready;

/* ------------------------------------------------------------------ */
/*  Apply the current MCU state to all actuators (called every cycle)  */
/* ------------------------------------------------------------------ */
static void apply_current_state(void)
{
	motor_set_angle(catch_arm_motor, arm_target);
	motor_set_mit(catch_dm_motor, 0.0f, dm_target, 0.0f);
	gpio_pin_set_dt(&catch_valve2, valve2_target);
	gpio_pin_set_dt(&catch_valve1, valve1_target);
}

/* ------------------------------------------------------------------ */
/*  Sequence handlers (blocking k_msleep — mirror R1_catch/main.c)     */
/* ------------------------------------------------------------------ */

/* ready4pluggin — Prepare 序列
 *   arm 45° (中间角度防飞出) → arm 90° → valve2 ON → DM 0° */
static void handle_prepare(void)
{
	LOG_INF("prepare: arm → 45°");
	arm_target = 45.0f;
	k_msleep(100);

	arm_target = 90.0f;
	LOG_INF("prepare: arm → 90°");
	k_msleep(600);

	valve2_target = true;
	gpio_pin_set_dt(&catch_valve2, true);
	LOG_INF("prepare: valve2 ON");
	k_msleep(500);

	dm_target = 0.0f;
	current_posture = POSTURE_P2;
	LOG_INF("prepare done → P2");
}

/* Raise — DM 90° */
static void handle_raise(void)
{
	dm_target = 90.0f;
	current_posture = POSTURE_P3;
	LOG_INF("raise: DM → 90° → P3");
}

/* ready4attack — Attack 序列
 *   valve2 OFF → arm 179.5° */
static void handle_attack(void)
{
	valve2_target = false;
	gpio_pin_set_dt(&catch_valve2, false);
	LOG_INF("attack: valve2 OFF");
	k_msleep(300);

	arm_target = 180.0f;
	dm_target = 90.0f;
	current_posture = POSTURE_P4;
	LOG_INF("attack: arm → 180.0°, DM → 90.0° → P4");
}

/* ------------------------------------------------------------------ */
/*  Init                                                                */
/* ------------------------------------------------------------------ */
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

	/* Initial state: P1 (idle) */
	arm_target   = 0.0f;
	dm_target    = 0.0f;
	valve2_target = false;
	valve1_target = false;
	apply_current_state();

	module_ready = true;
	LOG_INF("R1 catch module ready");
	return 0;
}

/* ------------------------------------------------------------------ */
/*  Update — called from main loop every ~5 ms                         */
/* ------------------------------------------------------------------ */
void module_r1_catch_update(const struct team_usb_packet *packet, bool packet_updated)
{
	if (!module_ready) {
		return;
	}

	if (packet_updated) {
		/* ---- Button 43 (Switch) — direct state mapping ---- */
		valve1_target = team_usb_button_pressed(packet, CTRL_BTN_SWITCH);

		/* ---- Button 40 (Prepare) — edge → ready4pluggin  ---- */
		{
			bool pressed = team_usb_button_pressed(packet, CTRL_BTN_PREPARE);
			if (pressed && !prepare_prev) {
				handle_prepare();
			}
			prepare_prev = pressed;
		}

		/* ---- Button 41 (Raise) — edge → DM 90°          ---- */
		{
			bool pressed = team_usb_button_pressed(packet, CTRL_BTN_RAISE);
			if (pressed && !raise_prev) {
				handle_raise();
			}
			raise_prev = pressed;
		}

		/* ---- Button 42 (Attack) — edge → ready4attack   ---- */
		{
			bool pressed = team_usb_button_pressed(packet, CTRL_BTN_ATTACK);
			if (pressed && !attack_prev) {
				handle_attack();
			}
			attack_prev = pressed;
		}
	}

	/* Drive hardware from MCU state (every cycle, regardless of packet) */
	apply_current_state();
}
