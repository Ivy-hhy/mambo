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
#define CTRL_BTN_PREPARE 40U
#define CTRL_BTN_RAISE   41U
#define CTRL_BTN_ATTACK  42U
#define CTRL_BTN_SWITCH  43U

/* Posture states — MCU-local, not dependent on button hold */
enum posture_state {
	POSTURE_P1,	/* idle:   arm=0°,    DM=0°,   valve2=off */
	POSTURE_P2,	/* ready:  arm=90°,   DM=90°,  valve2=on  */
	POSTURE_P3,	/* attack: arm=179.5°, DM=0°,   valve2=off */
	POSTURE_NUM,
};

struct posture {
	float arm_angle;
	float dm_angle;
	bool  valve2;
};

static const struct posture postures[POSTURE_NUM] = {
	[POSTURE_P1] = { .arm_angle = 0.0f,   .dm_angle = 0.0f,   .valve2 = false },
	[POSTURE_P2] = { .arm_angle = 90.0f,  .dm_angle = 90.0f,  .valve2 = true  },
	[POSTURE_P3] = { .arm_angle = 179.5f, .dm_angle = 0.0f,   .valve2 = false },
};

#define CATCH_ARM_NODE    DT_NODELABEL(motor1)
#define CATCH_DM_NODE     DT_NODELABEL(catch_dm_motor)
#define CATCH_VALVE_1_NODE DT_NODELABEL(catch_emvalve1)
#define CATCH_VALVE_2_NODE DT_NODELABEL(catch_emvalve2)

static const struct device    *catch_arm_motor = DEVICE_DT_GET(CATCH_ARM_NODE);
static const struct device    *catch_dm_motor  = DEVICE_DT_GET(CATCH_DM_NODE);
static const struct gpio_dt_spec catch_valve1  = GPIO_DT_SPEC_GET(CATCH_VALVE_1_NODE, gpios);
static const struct gpio_dt_spec catch_valve2  = GPIO_DT_SPEC_GET(CATCH_VALVE_2_NODE, gpios);

/* ---- MCU-persisted state (survives packet loss) ---- */
static enum posture_state current_posture = POSTURE_P1;
static bool              valve1_target    = false; /* true = energised */

/* ---- Edge-detection history ---- */
static bool switch_prev;
static bool prepare_prev;
static bool raise_prev;
static bool attack_prev;
static bool module_ready;

/* ------------------------------------------------------------------ */
/*  Apply the current MCU state to all actuators (called every cycle)  */
/* ------------------------------------------------------------------ */
static void apply_current_state(void)
{
	motor_set_angle(catch_arm_motor, postures[current_posture].arm_angle);
	motor_set_mit(catch_dm_motor, 0.0f, postures[current_posture].dm_angle, 0.0f);
	gpio_pin_set_dt(&catch_valve2, postures[current_posture].valve2);
	gpio_pin_set_dt(&catch_valve1, valve1_target);
}

/* ------------------------------------------------------------------ */
/*  Button handlers — update MCU state (all edge-triggered)  */
static void handle_switch(void)
{
	valve1_target = !valve1_target;
	LOG_INF("switch valve1 → %s", valve1_target ? "ON" : "OFF");
}
/* ------------------------------------------------------------------ */
static void handle_prepare(void)
{
	/* Toggle P1 ⟷ P2; from P3 go to P1 */
	if (current_posture == POSTURE_P2) {
		current_posture = POSTURE_P1;
	} else {
		current_posture = POSTURE_P2;
	}
	LOG_INF("prepare → posture %d", current_posture);
}

static void handle_raise(void)
{
	current_posture = POSTURE_P2;
	LOG_INF("raise → posture %d", current_posture);
}

static void handle_attack(void)
{
	current_posture = POSTURE_P3;
	LOG_INF("attack → posture %d", current_posture);
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

	/* Apply the initial posture (P1 / idle) so hardware matches state */
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

	/* ----- Phase 1 : process incoming packet → update MCU state ----- */
	if (packet_updated) {
		/* Button 1 — Switch: edge-triggered toggle valve1 ON/OFF */
		{
			bool switch_pressed = team_usb_button_pressed(packet, CTRL_BTN_SWITCH);
			if (switch_pressed && !switch_prev) {
				handle_switch();
			}
			switch_prev = switch_pressed;
		}

		/* Button 2 — Prepare: edge-triggered toggle P1 ⟷ P2 */
		bool prepare_pressed = team_usb_button_pressed(packet, CTRL_BTN_PREPARE);
		if (prepare_pressed && !prepare_prev) {
			handle_prepare();
		}
		prepare_prev = prepare_pressed;

		/* Button 3 — Raise: edge-triggered → P2 */
		bool raise_pressed = team_usb_button_pressed(packet, CTRL_BTN_RAISE);
		if (raise_pressed && !raise_prev) {
			handle_raise();
		}
		raise_prev = raise_pressed;

		/* Button 4 — Attack: edge-triggered → P3 */
		bool attack_pressed = team_usb_button_pressed(packet, CTRL_BTN_ATTACK);
		if (attack_pressed && !attack_prev) {
			handle_attack();
		}
		attack_prev = attack_pressed;
	}

	/* ----- Phase 2 : drive hardware from MCU state (every cycle) ----- */
	apply_current_state();
}
