#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>

#include "module_sbus2.h"

LOG_MODULE_REGISTER(team_usb_sbus2, LOG_LEVEL_INF);

#define CTRL_BTN_M3508_LEFT 32U
#define CTRL_BTN_M3508_CENTER 33U
#define CTRL_BTN_M3508_RIGHT 34U
#define CTRL_BTN_M2006_LEFT 35U
#define CTRL_BTN_M2006_RIGHT 36U
#define CTRL_BTN_RELAY_LEFT 37U
#define CTRL_BTN_RELAY_CENTER 38U
#define CTRL_BTN_RELAY_RIGHT 39U

#define M3508_LEFT_EXTEND_ANGLE 240.5f
#define M3508_LEFT_RETRACT_ANGLE -5.0f
#define M3508_CENTER_EXTEND_ANGLE -240.5f
#define M3508_CENTER_RETRACT_ANGLE 10.0f
#define M3508_RIGHT_EXTEND_ANGLE -240.5f
#define M3508_RIGHT_RETRACT_ANGLE 10.0f

#define M2006_LEFT_DOWN_ANGLE -170.0f
#define M2006_RIGHT_DOWN_ANGLE 170.0f
#define M2006_UP_ANGLE 0.0f

#define M3508_LEFT_NODE DT_NODELABEL(m3508_left)
#define M3508_CENTER_NODE DT_NODELABEL(m3508_center)
#define M3508_RIGHT_NODE DT_NODELABEL(m3508_right)
#define M2006_LEFT_NODE DT_NODELABEL(motor_o1)
#define M2006_RIGHT_NODE DT_NODELABEL(motor_o2)
#define RELAY_LEFT_NODE DT_NODELABEL(emvalve1)
#define RELAY_CENTER_NODE DT_NODELABEL(emvalve2)
#define RELAY_RIGHT_NODE DT_NODELABEL(emvalve3)

static const struct device *m3508_left = DEVICE_DT_GET(M3508_LEFT_NODE);
static const struct device *m3508_center = DEVICE_DT_GET(M3508_CENTER_NODE);
static const struct device *m3508_right = DEVICE_DT_GET(M3508_RIGHT_NODE);
static const struct device *m2006_left = DEVICE_DT_GET(M2006_LEFT_NODE);
static const struct device *m2006_right = DEVICE_DT_GET(M2006_RIGHT_NODE);
static const struct gpio_dt_spec relay_left = GPIO_DT_SPEC_GET(RELAY_LEFT_NODE, gpios);
static const struct gpio_dt_spec relay_center = GPIO_DT_SPEC_GET(RELAY_CENTER_NODE, gpios);
static const struct gpio_dt_spec relay_right = GPIO_DT_SPEC_GET(RELAY_RIGHT_NODE, gpios);

static bool module_ready;
static uint32_t log_count;

int module_sbus2_init(void)
{
	if (!device_is_ready(m3508_left) || !device_is_ready(m3508_center) ||
	    !device_is_ready(m3508_right) || !device_is_ready(m2006_left) ||
	    !device_is_ready(m2006_right)) {
		LOG_WRN("sbus_2 motors not ready, module disabled");
		return -ENODEV;
	}

	if (!device_is_ready(relay_left.port) || !device_is_ready(relay_center.port) ||
	    !device_is_ready(relay_right.port)) {
		LOG_WRN("sbus_2 relays not ready, module disabled");
		return -ENODEV;
	}

	motor_control(m3508_left, ENABLE_MOTOR);
	motor_control(m3508_center, ENABLE_MOTOR);
	motor_control(m3508_right, ENABLE_MOTOR);
	motor_control(m2006_left, ENABLE_MOTOR);
	motor_control(m2006_right, ENABLE_MOTOR);

	gpio_pin_configure_dt(&relay_left, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&relay_center, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&relay_right, GPIO_OUTPUT_INACTIVE);

	module_ready = true;
	LOG_INF("sbus_2 module ready");
	return 0;
}

void module_sbus2_update(const struct team_usb_packet *packet, bool packet_updated)
{
	bool m3508_left_extend;
	bool m3508_center_extend;
	bool m3508_right_extend;
	bool m2006_left_down;
	bool m2006_right_down;
	bool relay_left_on;
	bool relay_center_on;
	bool relay_right_on;
	float m3508_left_angle;
	float m3508_center_angle;
	float m3508_right_angle;
	float m2006_left_angle;
	float m2006_right_angle;

	if (!module_ready) {
		return;
	}

	m3508_left_extend = team_usb_button_pressed(packet, CTRL_BTN_M3508_LEFT);
	m3508_center_extend = team_usb_button_pressed(packet, CTRL_BTN_M3508_CENTER);
	m3508_right_extend = team_usb_button_pressed(packet, CTRL_BTN_M3508_RIGHT);
	m2006_left_down = team_usb_button_pressed(packet, CTRL_BTN_M2006_LEFT);
	m2006_right_down = team_usb_button_pressed(packet, CTRL_BTN_M2006_RIGHT);
	relay_left_on = team_usb_button_pressed(packet, CTRL_BTN_RELAY_LEFT);
	relay_center_on = team_usb_button_pressed(packet, CTRL_BTN_RELAY_CENTER);
	relay_right_on = team_usb_button_pressed(packet, CTRL_BTN_RELAY_RIGHT);

	m3508_left_angle = m3508_left_extend ? M3508_LEFT_EXTEND_ANGLE : M3508_LEFT_RETRACT_ANGLE;
	m3508_center_angle =
		m3508_center_extend ? M3508_CENTER_EXTEND_ANGLE : M3508_CENTER_RETRACT_ANGLE;
	m3508_right_angle = m3508_right_extend ? M3508_RIGHT_EXTEND_ANGLE : M3508_RIGHT_RETRACT_ANGLE;
	m2006_left_angle = m2006_left_down ? M2006_LEFT_DOWN_ANGLE : M2006_UP_ANGLE;
	m2006_right_angle = m2006_right_down ? M2006_RIGHT_DOWN_ANGLE : M2006_UP_ANGLE;

	motor_set_angle(m3508_left, m3508_left_angle);
	motor_set_angle(m3508_center, m3508_center_angle);
	motor_set_angle(m3508_right, m3508_right_angle);
	motor_set_angle(m2006_left, m2006_left_angle);
	motor_set_angle(m2006_right, m2006_right_angle);

	gpio_pin_set_dt(&relay_left, relay_left_on);
	gpio_pin_set_dt(&relay_center, relay_center_on);
	gpio_pin_set_dt(&relay_right, relay_right_on);

	if (!packet_updated || (log_count++ % 10U) != 0U) {
		return;
	}

	LOG_INF("sbus2 rx=%u vbtn=%d%d%d%d%d%d%d%d sticks L(%.3f, %.3f) R(%.3f, %.3f)",
		packet->rx_count,
		m3508_left_extend, m3508_center_extend, m3508_right_extend,
		m2006_left_down, m2006_right_down,
		relay_left_on, relay_center_on, relay_right_on,
		(double)packet->raw[TEAM_USB_IDX_LX], (double)packet->raw[TEAM_USB_IDX_LY],
		(double)packet->raw[TEAM_USB_IDX_RX], (double)packet->raw[TEAM_USB_IDX_RY]);
}
