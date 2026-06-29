#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/time_units.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/_intsup.h>

LOG_MODULE_REGISTER(robocon_integrated, LOG_LEVEL_INF);

/* ====================== SBUS 设备 ====================== */
#define SBUS_NODE DT_NODELABEL(sbus0)
const struct device *sbus = DEVICE_DT_GET(SBUS_NODE);

/* ====================== jiazhua 部分: 夹爪电机 ====================== */
#define MOTOR1_NODE DT_NODELABEL(motor01)
#define MOTOR2_NODE DT_NODELABEL(motor02)
#define DM_MOTOR_NODE DT_NODELABEL(dm_motor01)

static const struct device *motor01 = DEVICE_DT_GET(MOTOR1_NODE);
static const struct device *motor02 = DEVICE_DT_GET(MOTOR2_NODE);
static const struct device *dm_motor01 = DEVICE_DT_GET(DM_MOTOR_NODE);

#define TIMEOUT 100

static uint32_t last_move_time = 0;
static bool start_move = false;
static float dji_last = 0.0f;
static float dm_last = 0.0f;

/* ====================== hhy 部分: 升降电机 ====================== */
#define PI 3.1415926f
#define deg2high(deg)  ((deg) * PI * 45.0f / 360.0f)
#define high2deg(high) ((high) * 360.0f / (PI * 45.0f))

static float action[4] = {50.0f, 50.0f, 50.0f, 50.0f};
static int rank[4] = {1, 1, -1, -1};

const struct device *dmmotor[4] = {
    DEVICE_DT_GET(DT_NODELABEL(dm_motor000)),
    DEVICE_DT_GET(DT_NODELABEL(dm_motor001)),
    DEVICE_DT_GET(DT_NODELABEL(dm_motor002)),
    DEVICE_DT_GET(DT_NODELABEL(dm_motor003))
};

/* ====================== xipan 部分: 吸盘电机和电磁阀 ====================== */
#define M2006_NODE_0 DT_NODELABEL(motor_yq_o1)
#define M2006_NODE_1 DT_NODELABEL(motor_yq_o2)
#define M2006_NODE_2 DT_NODELABEL(motor_yq_o3)
#define M2006_NODE_3 DT_NODELABEL(motor_yq_o4)
#define DM_NODE_XIPAN DT_NODELABEL(dm_motor1)

static const struct device *m2006_motor_0 = DEVICE_DT_GET(M2006_NODE_0);
static const struct device *m2006_motor_1 = DEVICE_DT_GET(M2006_NODE_1);
static const struct device *m2006_motor_2 = DEVICE_DT_GET(M2006_NODE_2);
static const struct device *m2006_motor_3 = DEVICE_DT_GET(M2006_NODE_3);
static const struct device *dm_motor_xipan = DEVICE_DT_GET(DM_NODE_XIPAN);

#define EMValve1 DT_NODELABEL(emvalve1)
#define EMValve2 DT_NODELABEL(emvalve2)
#define EMValve3 DT_NODELABEL(emvalve3)
#define EMValve4 DT_NODELABEL(emvalve4)

const struct gpio_dt_spec emvalve1 = GPIO_DT_SPEC_GET(EMValve1, gpios);
const struct gpio_dt_spec emvalve2 = GPIO_DT_SPEC_GET(EMValve2, gpios);
const struct gpio_dt_spec emvalve3 = GPIO_DT_SPEC_GET(EMValve3, gpios);
const struct gpio_dt_spec emvalve4 = GPIO_DT_SPEC_GET(EMValve4, gpios);

/* ====================== 通用函数 ====================== */
static float deadband(float v)
{
    if (v > -0.05f && v < 0.05f)
        return 0.0f;
    return v;
}

/* ====================== jiazhua 模块: 夹爪控制 ====================== */
static void jiazhua_init(void)
{
    motor_control(motor01, SET_ZERO);
    motor_control(motor02, SET_ZERO);
    motor_control(dm_motor01, SET_ZERO);
    k_msleep(500);
    motor_control(dm_motor01, ENABLE_MOTOR);
    motor_set_mode(dm_motor01, MIT);
    LOG_INF("jiazhua: motors initialized");
}

static void jiazhua_control_m2006(float ch_dji)
{
    if (ch_dji != dji_last) {
        if (ch_dji > 0.0f) {
            motor_set_angle(motor01, 70.0f);
            motor_set_angle(motor02, 70.0f);
        } else if (ch_dji < 0.0f) {
            motor_set_angle(motor01, -60.0f);
            motor_set_angle(motor02, -60.0f);
        }
        start_move = true;
        last_move_time = k_uptime_get();
        dji_last = ch_dji;
    }
}

/* 超时检测与强制停止 */
static void jiazhua_check_timeout(void)
{
    if (start_move && (k_uptime_get() - last_move_time > TIMEOUT)) {
        motor_set_angle(motor01, 0.0f);
        motor_set_angle(motor02, 0.0f);
        start_move = false;
        motor_set_torque(motor01, 0.0f);
        motor_set_torque(motor02, 0.0f);
        LOG_INF("Motors stopped due to timeout");
    }
}

/* 根据角度反馈切换到力矩模式 */
static void jiazhua_check_angle_and_torque(float ch_dji)
{
    float angle_feedback = motor_get(motor01, ML_ANGLE);
    if (fabs(angle_feedback) < 70.0f && fabs(angle_feedback) > 60.0f) {
        motor_set_mode(motor01, ML_TORQUE);
        motor_set_mode(motor02, ML_TORQUE);
        if (ch_dji > 0.0f) {
            motor_set_torque(motor01, 0.5f);
            motor_set_torque(motor02, 0.5f);
        }
        LOG_INF("Angle feedback: %.2f, applying torque", angle_feedback);
    }
}

/* 达妙 DM 电机控制 */
static void jiazhua_control_dm(float ch_dm)
{
    if (ch_dm != dm_last) {
        if (ch_dm > 0.0f) {
            motor_set_mit(dm_motor01, 2.0f, -90.0f, -0.325f);
        } else if (ch_dm < 0.0f) {
            motor_set_mit(dm_motor01, 0.0f, 0.0f, 0.0f);
        } else {
            motor_set_mit(dm_motor01, 2.0f, -90.0f, -0.4f);
        }
        LOG_INF("DM Motor MIT set to: %s", ch_dm > 0.0f ? "Forward" : "Reverse");
        dm_last = ch_dm;
    }
}

/* ====================== hhy 模块: 升降控制 ====================== */
static void hhy_lifthigh(int id, float high)
{
    if (high > 400.0f) {
        high = 400.0f;
    } else if (high < 0.0f) {
        high = 0.0f;
    }

    if (!device_is_ready(dmmotor[id])) {
        LOG_ERR("hhy: motor %d not ready!", id);
        return;
    }

    float target_deg = -1.0f * rank[id] * high2deg(high);
    motor_set_mit(dmmotor[id], 20.0f, target_deg, 1.0f);
}

static void hhy_action_update(float sbus_percent)
{
    if (sbus_percent > 0.0f) {
        LOG_INF("hhy: SBUS11 Up: %f", sbus_percent);
        for (int i = 0; i < 4; i++) {
            action[i] += 1.0f;
            if (action[i] > 400.0f) action[i] = 400.0f;
            hhy_lifthigh(i, action[i]);
        }
    } else if (sbus_percent < 0.0f) {
        LOG_INF("hhy: SBUS11 Down: %f", sbus_percent);
        for (int i = 0; i < 4; i++) {
            action[i] -= 1.0f;
            if (action[i] < 0.0f) action[i] = 0.0f;
            hhy_lifthigh(i, action[i]);
        }
    }
}

static void hhy_init(void)
{
    for (int i = 0; i < 4; i++) {
        if (!device_is_ready(dmmotor[i])) {
            LOG_ERR("hhy: motor%d NOT READY!", i);
            continue;
        }
        LOG_INF("hhy: motor%d ready", i);
        motor_control(dmmotor[i], ENABLE_MOTOR);
        k_msleep(10);
        motor_set_mode(dmmotor[i], MIT);
        k_msleep(10);
        LOG_INF("hhy: motor%d enabled + MIT mode", i);
    }
}

/* ====================== xipan 模块: 吸盘控制 ====================== */
static void xipan_big_center_push(float sbus)
{
    if (sbus > 0.1f) {
        motor_set_angle(m2006_motor_2, -500.0f);
        gpio_pin_set_dt(&emvalve2, true);
        LOG_INF("xipan: big center push1");
    } else if (sbus < -0.1f) {
        motor_set_angle(m2006_motor_2, 0.0f);
        gpio_pin_set_dt(&emvalve2, false);
        LOG_INF("xipan: big center push2");
    }
}

static void xipan_left_push(float sbus)
{
    LOG_INF("xipan_left_push: sbus=%.3f", sbus);
    if (sbus < -0.1f) {
        motor_set_angle(m2006_motor_1, 0.0f);
        LOG_INF("xipan: left push - angle 0");
    } else if (sbus < 0.1f && sbus > -0.1f) {
        motor_set_angle(m2006_motor_1, 100.0f);
        gpio_pin_set_dt(&emvalve3, true);
        LOG_INF("xipan: left push - angle 100, valve ON");
    } else if (sbus > 0.1f) {
        motor_set_angle(m2006_motor_1, 360.0f);
        gpio_pin_set_dt(&emvalve3, false);
        LOG_INF("xipan: left push - angle 360, valve OFF");
    }
}

static void xipan_right_push(float sbus)
{
    LOG_INF("xipan_right_push: sbus=%.3f", sbus);
    if (sbus < -0.1f) {
        motor_set_angle(m2006_motor_0, 0.0f);
        LOG_INF("xipan: right push - angle 0");
    } else if (sbus < 0.1f && sbus > -0.1f) {
        motor_set_angle(m2006_motor_0, -100.0f);
        gpio_pin_set_dt(&emvalve4, true);
        LOG_INF("xipan: right push - angle -100, valve ON");
    } else if (sbus > 0.1f) {
        motor_set_angle(m2006_motor_0, -360.0f);
        gpio_pin_set_dt(&emvalve4, false);
        LOG_INF("xipan: right push - angle -360, valve OFF");
    }
}

static void xipan_center_xiqi(float sbus)
{
    if (sbus > 0.1f) {
        gpio_pin_set_dt(&emvalve1, true);
        LOG_INF("xipan: center xiqi1");
    } else {
        gpio_pin_set_dt(&emvalve1, false);
        LOG_INF("xipan: center xiqi2");
    }
}

static void xipan_init(void)
{
    motor_control(m2006_motor_0, ENABLE_MOTOR);
    motor_control(m2006_motor_1, ENABLE_MOTOR);
    motor_control(m2006_motor_2, ENABLE_MOTOR);
    motor_control(m2006_motor_3, ENABLE_MOTOR);
    motor_control(dm_motor_xipan, ENABLE_MOTOR);

    gpio_pin_configure_dt(&emvalve1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&emvalve2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&emvalve3, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&emvalve4, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(dm_motor_xipan)) {
        LOG_ERR("xipan: DM motor device NOT ready!");
    } else {
        LOG_INF("xipan: DM motor device is ready");
    }

    k_msleep(300);
    motor_set_mode(dm_motor_xipan, MIT);
    motor_control(dm_motor_xipan, ENABLE_MOTOR);
    k_msleep(100);
    motor_set_mit(dm_motor_xipan, 2, 180, 0.0f);
    LOG_INF("xipan: initialized, DM motor set to 180 deg");
}

/* ====================== 主函数 ====================== */
int main(void)
{
    k_msleep(500);
    LOG_INF("=== Robocon Integrated Demo Start ===");

    /* 初始化各模块 */
    jiazhua_init();
    hhy_init();
    xipan_init();

    LOG_INF("All modules initialized");

    /* SBUS 通道历史值 */
    static float last_ch9 = 0, last_ch8 = 0;
    static float last_ch4 = 0, last_ch7 = 0, last_ch11 = 0;

    while (1) {
        /* 读取 SBUS 通道值 */
        float ch9 = deadband(sbus_get_percent(sbus, 9));   /* jiazhua DM */
        float ch8 = deadband(sbus_get_percent(sbus, 8));   /* jiazhua M2006 */
        float ch5 = deadband(sbus_get_percent(sbus, 5));   /* xipan center push */
        float ch4 = deadband(sbus_get_percent(sbus, 4));   /* xipan 左推 */
        float ch7 = deadband(sbus_get_percent(sbus, 7));   /* xipan 右推 */
        float ch11 = deadband(sbus_get_percent(sbus, 11)); /* hhy 升降 */

        /* jiazhua 模块处理 */
        if (ch9 != last_ch9) {
            jiazhua_control_dm(ch9);
            last_ch9 = ch9;
        }
        if (ch8 != last_ch8) {
            jiazhua_control_m2006(ch8);
            last_ch8 = ch8;
        }
        jiazhua_check_timeout();

        k_msleep(200);

        jiazhua_check_angle_and_torque(ch8);

        /* hhy 模块处理 */
        if (ch11 != last_ch11) {
            hhy_action_update(ch11);
            last_ch11 = ch11;
        }

        /* xipan 模块处理 */
        if (ch4 != last_ch4) {
            xipan_left_push(ch4);
            last_ch4 = ch4;
        }
        if (ch7 != last_ch7) {
            xipan_right_push(ch7);
            last_ch7 = ch7;
        }

        /* 大中心推 + 中吸 */
        xipan_big_center_push(ch5);
        xipan_center_xiqi(ch8);

        k_msleep(500);
    }