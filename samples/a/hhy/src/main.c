#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/gpio.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/dual/dual_protocol.h>

#include "devices.h"

LOG_MODULE_REGISTER(lift_demo, LOG_LEVEL_INF);

#define CTRL_SYNC_ID 0x0303
#define CTRL_FLOAT_NUM 7
#define CTRL_BUTTON_MASK_NUM 3
#define CTRL_BUTTON_A_ID 3
#define CTRL_BUTTON_Y_ID 6

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
static float ctrl_latest[CTRL_FLOAT_NUM];
static uint32_t ctrl_rx_count;

K_SEM_DEFINE(ctrl_rx_sem, 0, 1);

float action[4] = {0.0f, 0.0f, 0.0f, 0.0f};

int rank[4] = {-1, -1, 1, 1};
static const float motor_ff[4] = {0.5f, 1.8f, 0.5f, 1.8f};

const struct device *dm_motor[4];

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

static void control_rx_cb(int status)
{
    if (status != SYNC_PACK_STATUS_DONE) {
        return;
    }

    memcpy(ctrl_data, ctrl_rx, sizeof(ctrl_data));
    ctrl_rx_count++;
    k_sem_give(&ctrl_rx_sem);
}

void lifthigh(int id, float target_deg)
{
    float torque_ff = 0.0f;

    if (!device_is_ready(dm_motor[id])) {
        LOG_ERR("motor %d not ready!", id);
        return;
    }

    if (target_deg > 0.0f) {
        torque_ff = motor_ff[id];
    } else if (target_deg < 0.0f) {
        torque_ff = -motor_ff[id];
    }

    motor_set_mit(dm_motor[id], 0.0f, target_deg, torque_ff);
}

static void hold_current_target(void)
{
    for (int i = 0; i < 4; i++) {
        lifthigh(i, action[i]);
    }
}

static const float gear_angle_ac[] = {
    0.0f,
    360.0f,
    720.0f,
    1440.0f,
    3600.0f,
    4800.0f,
};

static const float gear_angle_bd[] = {
    0.0f,
    410.0f,
    770.0f,
    1490.0f,
    3650.0f,
    4850.0f,
};

#define MAX_GEAR ARRAY_SIZE(gear_angle_ac)

static int current_gear = 0;
static bool last_a_pressed;
static bool last_y_pressed;

static void apply_current_gear(void)
{
    float target_angle_ac = gear_angle_ac[current_gear];
    float target_angle_bd = gear_angle_bd[current_gear];

    action[0] = target_angle_ac * rank[0];
    action[1] = target_angle_bd * rank[1];
    action[2] = target_angle_ac * rank[2];
    action[3] = target_angle_bd * rank[3];

    for (int i = 0; i < 4; i++) {
        lifthigh(i, action[i]);
    }
}

static void gear_step(int delta)
{
    int prev_gear = current_gear;

    if (delta > 0 && current_gear < MAX_GEAR - 1) {
        current_gear++;
    } else if (delta < 0 && current_gear > 0) {
        current_gear--;
    }

    if (current_gear == prev_gear) {
        LOG_INF("Gear unchanged: %d/%d", current_gear + 1, MAX_GEAR);
        return;
    }

    apply_current_gear();

    if (delta > 0) {
        LOG_INF("Y pressed -> Gear Up: %d/%d, Angle: %.1f deg",
                current_gear + 1, MAX_GEAR, (double)gear_angle_ac[current_gear]);
    } else {
        LOG_INF("A pressed -> Gear Down: %d/%d, Angle: %.1f deg",
                current_gear + 1, MAX_GEAR, (double)gear_angle_ac[current_gear]);
    }
}

void prepare(void)
{
    for (int i = 0; i < 4; i++) {
        if (!device_is_ready(dm_motor[i])) {
            LOG_ERR("motor%d NOT READY!", i);
            continue;
        }

        LOG_INF("motor%d ready", i);

        motor_control(dm_motor[i], ENABLE_MOTOR);
        k_msleep(10);

        motor_set_mode(dm_motor[i], MIT);
        k_msleep(10);

        LOG_INF("motor%d enabled + MIT mode", i);
    }
}

int main(void)
{
    int ret;
    sync_table_t *control_rx;

    k_msleep(200);

    LOG_INF("=== Lift Demo Start ===");

    dm_motor[0] = dm_motor000;
    dm_motor[1] = dm_motor001;
    dm_motor[2] = dm_motor002;
    dm_motor[3] = dm_motor003;

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

    LOG_INF("USB control ready: sync=0x%04x, Y=up, A=down", CTRL_SYNC_ID);

    prepare();
    apply_current_gear();
    hold_current_target();

    LOG_INF("Start at gear 1/%d, waiting for USB buttons.", MAX_GEAR);

    while (1) {
        uint16_t masks[CTRL_BUTTON_MASK_NUM];
        bool a_pressed;
        bool y_pressed;

        if (k_sem_take(&ctrl_rx_sem, K_NO_WAIT) == 0) {
            memcpy(ctrl_latest, ctrl_data, sizeof(ctrl_latest));
        }

        masks[0] = ctrl_float_to_mask(ctrl_latest[CTRL_IDX_BUTTONS_0_15]);
        masks[1] = ctrl_float_to_mask(ctrl_latest[CTRL_IDX_BUTTONS_16_31]);
        masks[2] = ctrl_float_to_mask(ctrl_latest[CTRL_IDX_BUTTONS_32_47]);

        a_pressed = ctrl_button_pressed(masks, CTRL_BUTTON_A_ID);
        y_pressed = ctrl_button_pressed(masks, CTRL_BUTTON_Y_ID);

        if (y_pressed && !last_y_pressed) {
            gear_step(1);
        }

        if (a_pressed && !last_a_pressed) {
            gear_step(-1);
        }

        last_a_pressed = a_pressed;
        last_y_pressed = y_pressed;

        hold_current_target();
        k_msleep(5);
    }

    return 0;
}
