#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/kernel.h>

#define MOTOR_NODE_1 DT_INST(0, rs_motor)
#define MOTOR_NODE_2 DT_INST(1, rs_motor)
#define SBUS_NODE    DT_NODELABEL(sbus0)

#define FEEDBACK_TIMEOUT_MS     2000
#define COMMAND_PERIOD_MS       5
#define ZERO_SETTLE_MS          200
#define RISE_STEP_DEG           2.0f
#define RISE_INTERVAL_MS        40
#define DESCENT_STEP_DEG        2.0f
#define DESCENT_SETTLE_MS       400
#define TARGET_HOLD_MS          800

#define LIFT_STROKE_DEG         180.0f
#define LIFT_TORQUE_NM          3.5f
#define DESCENT_TORQUE_NM       17.0f

#define MAX_GEAR                5
#define SBUS_DEADZONE           0.05f
#define SBUS_TRIGGER_COUNT      2

struct lift_motor {
	const char *name;
	const struct device *dev;
	float direction;
};

static const struct lift_motor lift_motors[] = {
	{
		.name = "motor_1",
		.dev = DEVICE_DT_GET(MOTOR_NODE_1),
		.direction = 1.0f,
	},
	{
		.name = "motor_2",
		.dev = DEVICE_DT_GET(MOTOR_NODE_2),
		.direction = -1.0f,
	},
};

static const struct device *const sbus = DEVICE_DT_GET(SBUS_NODE);

static const float gear_lift_deg[MAX_GEAR] = {
	0.0f,
	45.0f,
	90.0f,
	135.0f,
	LIFT_STROKE_DEG,
};

static float current_lift_target_deg;
static float last_sbus11;
static int current_gear = -1;
static int sbus11_change_count;

static int wait_feedback_ready(const struct lift_motor *motor, int32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	motor_status_t status;

	while (k_uptime_get() < deadline) {
		if (motor_get(motor->dev, &status) == 0) {
			return 0;
		}

		k_msleep(50);
	}

	return -ETIMEDOUT;
}

static float signed_torque(float target_deg, float torque_nm)
{
	if (fabsf(target_deg) < 0.01f) {
		return 0.0f;
	}

	return copysignf(torque_nm, target_deg);
}

static void send_lift_target(float lift_deg, float torque_nm)
{
	for (size_t i = 0; i < ARRAY_SIZE(lift_motors); ++i) {
		float target_deg = lift_motors[i].direction * lift_deg;
		float target_torque = signed_torque(target_deg, torque_nm);

		motor_set_mit(lift_motors[i].dev, 0.0f, target_deg, target_torque);
	}
}

static void hold_lift_target(float lift_deg, float torque_nm, int32_t hold_ms)
{
	int64_t deadline = k_uptime_get() + hold_ms;

	while (k_uptime_get() < deadline) {
		send_lift_target(lift_deg, torque_nm);
		k_msleep(COMMAND_PERIOD_MS);
	}
}

static int init_lift_motor(const struct lift_motor *motor)
{
	if (!device_is_ready(motor->dev)) {
		return -ENODEV;
	}

	motor_control(motor->dev, ENABLE_MOTOR);
	k_msleep(100);

	motor_control(motor->dev, AUTO_REPORT_ENABLE);
	if (wait_feedback_ready(motor, FEEDBACK_TIMEOUT_MS) < 0) {
		return -ETIMEDOUT;
	}

	motor_control(motor->dev, SET_ZERO);
	k_msleep(ZERO_SETTLE_MS);

	motor_set_mode(motor->dev, MIT);
	k_msleep(50);

	return 0;
}

static float clamp_step_target(float current, float target, float step)
{
	float next = current + step;

	if ((step > 0.0f && next > target) || (step < 0.0f && next < target)) {
		return target;
	}

	return next;
}

static void move_lift_to(float target_lift_deg)
{
	float delta = target_lift_deg - current_lift_target_deg;
	float step = (delta >= 0.0f) ? RISE_STEP_DEG : -DESCENT_STEP_DEG;
	float torque = (delta >= 0.0f) ? LIFT_TORQUE_NM : DESCENT_TORQUE_NM;
	int32_t settle_ms = (delta >= 0.0f) ? RISE_INTERVAL_MS : DESCENT_SETTLE_MS;

	if (fabsf(delta) <= 0.01f) {
		hold_lift_target(target_lift_deg, LIFT_TORQUE_NM, TARGET_HOLD_MS);
		return;
	}

	while (fabsf(target_lift_deg - current_lift_target_deg) > 0.01f) {
		current_lift_target_deg =
			clamp_step_target(current_lift_target_deg, target_lift_deg, step);
		hold_lift_target(current_lift_target_deg, torque, settle_ms);
	}

	hold_lift_target(target_lift_deg, torque, TARGET_HOLD_MS);
}

static int gear_step_from_sbus(float sbus10)
{
	if (sbus10 > SBUS_DEADZONE) {
		return 1;
	}

	if (sbus10 < -SBUS_DEADZONE) {
		return -1;
	}

	return 0;
}

static void gear_control_update(float sbus10, float sbus11)
{
	int gear_step;

	if (fabsf(sbus11 - last_sbus11) <= SBUS_DEADZONE) {
		return;
	}

	last_sbus11 = sbus11;
	sbus11_change_count++;

	if (sbus11_change_count < SBUS_TRIGGER_COUNT) {
		return;
	}

	sbus11_change_count = 0;
	gear_step = gear_step_from_sbus(sbus10);
	if (gear_step == 0) {
		return;
	}

	if (gear_step > 0 && current_gear < MAX_GEAR - 1) {
		current_gear++;
	} else if (gear_step < 0 && current_gear > 0) {
		current_gear--;
	} else if (current_gear < 0) {
		return;
	}

	move_lift_to(gear_lift_deg[current_gear]);
}

static void stop_lift_pair(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(lift_motors); ++i) {
		motor_set_mit(lift_motors[i].dev, 0.0f, 0.0f, 0.0f);
		motor_control(lift_motors[i].dev, DISABLE_MOTOR);
	}
}

int main(void)
{
	if (!device_is_ready(sbus)) {
		return -ENODEV;
	}

	for (size_t i = 0; i < ARRAY_SIZE(lift_motors); ++i) {
		if (init_lift_motor(&lift_motors[i]) < 0) {
			stop_lift_pair();
			return -EIO;
		}
	}

	current_lift_target_deg = 0.0f;
	last_sbus11 = sbus_get_percent(sbus, 11);

	while (1) {
		float sbus10 = sbus_get_percent(sbus, 10);
		float sbus11 = sbus_get_percent(sbus, 11);

		gear_control_update(sbus10, sbus11);
		k_msleep(COMMAND_PERIOD_MS);
	}

	return 0;
}
