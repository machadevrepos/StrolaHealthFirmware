#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "bmi270.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BMI270_NODE DT_NODELABEL(bmi270)

#if !DT_NODE_HAS_STATUS(BMI270_NODE, okay)
#error "BMI270 node is not enabled in devicetree overlay"
#endif

/* Active mode */
#define SENSOR_SAMPLE_INTERVAL_MS     50
#define BLE_SEND_INTERVAL_MS          1000

/* Software sleep mode */
#define NO_MOTION_TIMEOUT_MS          60000
#define NO_MOTION_THRESHOLD_MG        80
#define WAKE_DELTA_THRESHOLD_MG       25
#define SLEEP_SAMPLE_INTERVAL_MS      200
#define WAKE_HIT_COUNT_REQUIRED       1

/* Flash saving */
#define STEP_SAVE_EVERY_N_STEPS       5
#define STEP_SAVE_TIME_MS             30000

/* BMI270 +/-2g sensitivity */
#define BMI270_ACC_LSB_PER_G          16384
#define AIRPLANE_MAX_MINUTES   1440   /* 24 h cap */

static volatile bool airplane_mode = false;
static volatile uint32_t airplane_req_ms = 0;

static struct i2c_dt_spec bmi270_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
static struct bmi2_dev bmi270_dev;
static bool bmi270_ready = false;

static struct bt_conn *current_conn;
static bool notify_enabled = false;
static bool advertising_active = false;
static bool sleep_mode = false;

/* BLE advertising work item */
static struct k_work adv_work;

static atomic_t step_count;

/* BMI270 hardware step counter tracking */
static uint32_t bmi270_step_base = 0;
static uint32_t bmi270_hw_steps_last = 0;

static bool gravity_initialized = false;
static int32_t gravity_x = 0;
static int32_t gravity_y = 0;
static int32_t gravity_z = 0;

static uint32_t step_counter_start_ms = 0;
static uint32_t last_motion_time_ms = 0;
static uint32_t last_ble_send_ms = 0;

static uint8_t wake_motion_hits = 0;
static int32_t last_motion_mg = 0;

static bool step_storage_ready = false;
static uint32_t last_saved_step_count = 0;
static uint32_t last_flash_save_time_ms = 0;

/* Sleep wake reference */
static bool sleep_ref_initialized = false;
static int32_t sleep_ref_ax = 0;
static int32_t sleep_ref_ay = 0;
static int32_t sleep_ref_az = 0;
static int32_t sleep_wake_delta_mg = 0;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* ---------- Bosch BMI270 I2C bridge ---------- */

static BMI2_INTF_RETURN_TYPE bosch_i2c_read(uint8_t reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr)
{
    const struct i2c_dt_spec *i2c = (const struct i2c_dt_spec *)intf_ptr;
    int err;

    err = i2c_write_read_dt(i2c, &reg_addr, 1, reg_data, len);

    return (err == 0) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE bosch_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len,
                                             void *intf_ptr)
{
    const struct i2c_dt_spec *i2c = (const struct i2c_dt_spec *)intf_ptr;
    uint8_t tx_buf[65];
    int err;

    if ((len + 1U) > sizeof(tx_buf)) {
        return BMI2_E_COM_FAIL;
    }

    tx_buf[0] = reg_addr;
    memcpy(&tx_buf[1], reg_data, len);

    err = i2c_write_dt(i2c, tx_buf, len + 1U);

    return (err == 0) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static void bosch_delay_us(uint32_t period_us, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_busy_wait(period_us);
}

/* ---------- BMI270 ---------- */

static int bmi270_probe_i2c_address(void)
{
    uint8_t chip_id = 0;
    uint16_t try_addr[3] = { bmi270_i2c.addr, 0x68, 0x69 };

    if (!device_is_ready(bmi270_i2c.bus)) {
        printk("BMI270 I2C bus is not ready\n");
        return -ENODEV;
    }

    for (int i = 0; i < ARRAY_SIZE(try_addr); i++) {
        bool duplicate = false;

        for (int j = 0; j < i; j++) {
            if (try_addr[i] == try_addr[j]) {
                duplicate = true;
                break;
            }
        }

        if (duplicate) {
            continue;
        }

        bmi270_i2c.addr = try_addr[i];

        if (i2c_reg_read_byte_dt(&bmi270_i2c, BMI2_CHIP_ID_ADDR, &chip_id) == 0) {
            printk("BMI270 probe address 0x%02X, CHIP_ID=0x%02X\n",
                   bmi270_i2c.addr,
                   chip_id);

            if (chip_id == BMI270_CHIP_ID) {
                return 0;
            }
        }
    }

    printk("BMI270 not found at 0x68 or 0x69\n");
    return -ENODEV;
}

static int bmi270_reset_hw_step_counter(void)
{
    int8_t rslt;
    struct bmi2_sens_config step_cfg;

    if (!bmi270_ready) {
        return -ENODEV;
    }

    memset(&step_cfg, 0, sizeof(step_cfg));
    step_cfg.type = BMI2_STEP_COUNTER;

    rslt = bmi270_get_sensor_config(&step_cfg, 1, &bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("BMI270 get step config failed: %d\n", rslt);
        return -EIO;
    }

    step_cfg.cfg.step_counter.reset_counter = 1;

    rslt = bmi270_set_sensor_config(&step_cfg, 1, &bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("BMI270 step counter reset failed: %d\n", rslt);
        return -EIO;
    }

    k_sleep(K_MSEC(50));

    step_cfg.cfg.step_counter.reset_counter = 0;

    rslt = bmi270_set_sensor_config(&step_cfg, 1, &bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("BMI270 step counter reset clear failed: %d\n", rslt);
        return -EIO;
    }

    bmi270_hw_steps_last = 0;

    printk("BMI270 hardware step counter reset\n");

    return 0;
}

static int bmi270_read_hw_step_counter(uint32_t *hw_steps)
{
    int8_t rslt;
    struct bmi2_feat_sensor_data feature_data;

    if (!bmi270_ready) {
        return -ENODEV;
    }

    memset(&feature_data, 0, sizeof(feature_data));
    feature_data.type = BMI2_STEP_COUNTER;

    rslt = bmi270_get_feature_data(&feature_data, 1, &bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("BMI270 get step counter failed: %d\n", rslt);
        return -EIO;
    }

    *hw_steps = feature_data.sens_data.step_counter_output;

    return 0;
}

static int bmi270_bosch_init_sensor(void)
{
    int8_t rslt;
    uint8_t sens_list[2] = {
        BMI2_ACCEL,
        BMI2_STEP_COUNTER
    };
    struct bmi2_sens_config acc_cfg;
    int err;

    bmi270_ready = false;

    err = bmi270_probe_i2c_address();
    if (err) {
        return err;
    }

    memset(&bmi270_dev, 0, sizeof(bmi270_dev));

    bmi270_dev.intf = BMI2_I2C_INTF;
    bmi270_dev.read = bosch_i2c_read;
    bmi270_dev.write = bosch_i2c_write;
    bmi270_dev.delay_us = bosch_delay_us;
    bmi270_dev.intf_ptr = &bmi270_i2c;
    bmi270_dev.read_write_len = 32;

    rslt = bmi270_init(&bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("bmi270_init failed: %d\n", rslt);
        return -EIO;
    }

    acc_cfg.type = BMI2_ACCEL;
    acc_cfg.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    acc_cfg.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    acc_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    acc_cfg.cfg.acc.range = BMI2_ACC_RANGE_2G;

    rslt = bmi270_set_sensor_config(&acc_cfg, 1, &bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("BMI270 accel config failed: %d\n", rslt);
        return -EIO;
    }

    rslt = bmi270_sensor_enable(sens_list, 2, &bmi270_dev);
    if (rslt != BMI2_OK) {
        printk("BMI270 accel/step counter enable failed: %d\n", rslt);
        return -EIO;
    }

    k_sleep(K_MSEC(100));

    bmi270_ready = true;

    bmi270_reset_hw_step_counter();

    bmi270_step_base = (uint32_t)atomic_get(&step_count);
    bmi270_hw_steps_last = 0;

    printk("BMI270 Bosch API ready at I2C address 0x%02X\n", bmi270_i2c.addr);

    return 0;
}

static bool bmi270_accel_is_valid(int32_t ax_mg, int32_t ay_mg, int32_t az_mg)
{
    if ((ax_mg == 0) && (ay_mg == 0) && (az_mg == 0)) {
        return false;
    }

    return true;
}

static int bmi270_read_values(int32_t *ax_mg,
                              int32_t *ay_mg,
                              int32_t *az_mg)
{
    int8_t rslt;
    struct bmi2_sens_data sensor_data;

    if (!bmi270_ready) {
        return -ENODEV;
    }

    memset(&sensor_data, 0, sizeof(sensor_data));

    rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
    if (rslt != BMI2_OK) {
        return -EIO;
    }

    *ax_mg = ((int32_t)sensor_data.acc.x * 1000) / BMI270_ACC_LSB_PER_G;
    *ay_mg = ((int32_t)sensor_data.acc.y * 1000) / BMI270_ACC_LSB_PER_G;
    *az_mg = ((int32_t)sensor_data.acc.z * 1000) / BMI270_ACC_LSB_PER_G;

    return 0;
}

static void bmi270_update_step_count_from_hw(void)
{
    int err;
    uint32_t hw_steps;
    uint32_t total_steps;
    uint32_t old_steps;

    err = bmi270_read_hw_step_counter(&hw_steps);
    if (err) {
        return;
    }

    bmi270_hw_steps_last = hw_steps;

    total_steps = bmi270_step_base + hw_steps;
    old_steps = (uint32_t)atomic_get(&step_count);

    if (total_steps > old_steps) {
        atomic_set(&step_count, (atomic_val_t)total_steps);

        printk("BMI270 STEP updated. HW:%lu Total:%lu\n",
               (unsigned long)hw_steps,
               (unsigned long)total_steps);
    }

    if (total_steps < old_steps) {
        bmi270_step_base = old_steps;
        bmi270_hw_steps_last = 0;
    }
}

/* ---------- Permanent flash storage ---------- */

static int steps_settings_set(const char *name,
                              size_t len,
                              settings_read_cb read_cb,
                              void *cb_arg)
{
    const char *next;
    uint32_t saved_steps = 0;
    ssize_t rc;

    if (settings_name_steq(name, "count", &next) && !next) {
        if (len != sizeof(saved_steps)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, &saved_steps, sizeof(saved_steps));
        if (rc < 0) {
            return (int)rc;
        }

        if (rc != sizeof(saved_steps)) {
            return -EINVAL;
        }

        atomic_set(&step_count, (atomic_val_t)saved_steps);
        last_saved_step_count = saved_steps;

        printk("Loaded saved step count from flash: %lu\n",
               (unsigned long)saved_steps);

        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(step_storage,
                               "steps",
                               NULL,
                               steps_settings_set,
                               NULL,
                               NULL);

static int step_storage_init(void)
{
    int err;

    err = settings_subsys_init();
    if (err) {
        printk("Settings init failed: %d\n", err);
        return err;
    }

    err = settings_load_subtree("steps");
    if (err) {
        printk("Step settings load failed: %d\n", err);
        return err;
    }

    step_storage_ready = true;
    last_saved_step_count = (uint32_t)atomic_get(&step_count);
    last_flash_save_time_ms = k_uptime_get_32();

    printk("Step storage ready. Current steps = %lu\n",
           (unsigned long)last_saved_step_count);

    return 0;
}

static int save_step_count_to_flash(void)
{
    uint32_t steps;
    int err;

    if (!step_storage_ready) {
        return -EAGAIN;
    }

    steps = (uint32_t)atomic_get(&step_count);

    if (steps == last_saved_step_count) {
        return 0;
    }

    err = settings_save_one("steps/count", &steps, sizeof(steps));
    if (err) {
        printk("Failed to save steps to flash: %d\n", err);
        return err;
    }

    last_saved_step_count = steps;
    last_flash_save_time_ms = k_uptime_get_32();

    printk("Saved step count to flash: %lu\n", (unsigned long)steps);

    return 0;
}

static void save_step_count_if_needed(void)
{
    uint32_t steps = (uint32_t)atomic_get(&step_count);
    uint32_t now_ms = k_uptime_get_32();

    if (!step_storage_ready) {
        return;
    }

    if (steps != last_saved_step_count) {
        if (steps > last_saved_step_count &&
            (steps - last_saved_step_count) >= STEP_SAVE_EVERY_N_STEPS) {
            save_step_count_to_flash();
            return;
        }

        if ((uint32_t)(now_ms - last_flash_save_time_ms) >= STEP_SAVE_TIME_MS) {
            save_step_count_to_flash();
            return;
        }
    }
}

/* ---------- Runtime state ---------- */

static void init_runtime_state_only(void)
{
    gravity_initialized = false;
    gravity_x = 0;
    gravity_y = 0;
    gravity_z = 0;

    last_motion_mg = 0;

    sleep_mode = false;
    wake_motion_hits = 0;

    sleep_ref_initialized = false;
    sleep_wake_delta_mg = 0;

    step_counter_start_ms = k_uptime_get_32();
    last_motion_time_ms = step_counter_start_ms;
    last_ble_send_ms = step_counter_start_ms;
}

static void reset_step_counter(bool save_to_flash)
{
    atomic_set(&step_count, 0);
    init_runtime_state_only();

    if (bmi270_ready) {
        bmi270_reset_hw_step_counter();
    }

    bmi270_step_base = 0;
    bmi270_hw_steps_last = 0;

    if (save_to_flash && step_storage_ready) {
        last_saved_step_count = 1;
        save_step_count_to_flash();
    }

    printk("Step counter reset\n");
}

/* ---------- BLE advertising ---------- */



static int start_ble_advertising(void)
{
    int err;

    if (airplane_mode) {
    printk("Airplane mode active, advertising blocked\n");
    return 0;
    }


    if (advertising_active) {
        return 0;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                          ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));

    if (err) {
        printk("Advertising start failed: %d\n", err);
        return err;
    }

    advertising_active = true;
    printk("Advertising as %s\n", DEVICE_NAME);

    return 0;
}

static void adv_work_handler(struct k_work *work)
{
    if (!sleep_mode) {
        start_ble_advertising();
    }
}

static void stop_ble_advertising(void)
{
    int err;

    if (!advertising_active) {
        return;
    }

    err = bt_le_adv_stop();

    if (err) {
        printk("Advertising stop failed: %d\n", err);
    } else {
        printk("Advertising stopped\n");
    }

    advertising_active = false;
}

/* ---------- Motion only for sleep/wake ---------- */

static int32_t calculate_motion_mg(int32_t ax_mg,
                                   int32_t ay_mg,
                                   int32_t az_mg)
{
    int32_t linear_x;
    int32_t linear_y;
    int32_t linear_z;
    int32_t motion_mg;

    if (!gravity_initialized) {
        gravity_x = ax_mg;
        gravity_y = ay_mg;
        gravity_z = az_mg;
        gravity_initialized = true;
        return 0;
    }

    gravity_x += (ax_mg - gravity_x) / 16;
    gravity_y += (ay_mg - gravity_y) / 16;
    gravity_z += (az_mg - gravity_z) / 16;

    linear_x = ax_mg - gravity_x;
    linear_y = ay_mg - gravity_y;
    linear_z = az_mg - gravity_z;

    motion_mg = abs32(linear_x) + abs32(linear_y) + abs32(linear_z);
    last_motion_mg = motion_mg;

    return motion_mg;
}

static bool small_motion_detected_in_sleep(int32_t ax_mg,
                                           int32_t ay_mg,
                                           int32_t az_mg)
{
    int32_t delta_x;
    int32_t delta_y;
    int32_t delta_z;
    int32_t delta_sum;

    if (!sleep_ref_initialized) {
        sleep_ref_ax = ax_mg;
        sleep_ref_ay = ay_mg;
        sleep_ref_az = az_mg;
        sleep_ref_initialized = true;
        sleep_wake_delta_mg = 0;
        return false;
    }

    delta_x = abs32(ax_mg - sleep_ref_ax);
    delta_y = abs32(ay_mg - sleep_ref_ay);
    delta_z = abs32(az_mg - sleep_ref_az);

    delta_sum = delta_x + delta_y + delta_z;
    sleep_wake_delta_mg = delta_sum;

    if (delta_sum < WAKE_DELTA_THRESHOLD_MG) {
        sleep_ref_ax += (ax_mg - sleep_ref_ax) / 8;
        sleep_ref_ay += (ay_mg - sleep_ref_ay) / 8;
        sleep_ref_az += (az_mg - sleep_ref_az) / 8;
        return false;
    }

    return true;
}

/* ---------- BLE send ---------- */

static int ble_send_text(const char *text)
{
    int err;

    if (!current_conn || !notify_enabled) {
        return -ENOTCONN;
    }

    for (int retry = 0; retry < 3; retry++) {
        err = bt_nus_send(current_conn, text, strlen(text));

        if (err == 0) {
            return 0;
        }

        if (err == -ENOMEM) {
            k_sleep(K_MSEC(100));
            continue;
        }

        return err;
    }

    return -ENOMEM;
}

static void send_step_count_to_mobile(void)
{
    char tx_buf[20];
    int err;

    snprintk(tx_buf, sizeof(tx_buf), "ST:%ld\n", (long)atomic_get(&step_count));

    err = ble_send_text(tx_buf);

    if (err == 0) {
        printk("BLE sent: %s", tx_buf);
    } else if (err == -ENOTCONN) {
        /* Mobile not connected or TX notify not enabled. */
    } else {
        printk("BLE send failed: %d\n", err);
    }
}

/* ---------- Sleep mode ---------- */

static void enter_sleep_mode(void)
{
    if (sleep_mode) {
        return;
    }

    sleep_mode = true;
    wake_motion_hits = 0;
    sleep_ref_initialized = false;
    sleep_wake_delta_mg = 0;

    printk("\nNo motion for 60 seconds. Entering sleep mode...\n");

    save_step_count_to_flash();

    if (current_conn && notify_enabled) {
        ble_send_text("SLP\n");
        k_sleep(K_MSEC(200));
    }

    if (current_conn) {
        bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    } else {
        stop_ble_advertising();
    }

    printk("Sleep mode active. BMI270 is still sampled every %d ms.\n",
           SLEEP_SAMPLE_INTERVAL_MS);
}

static void wake_from_sleep_mode(void)
{
    if (!sleep_mode) {
        return;
    }

    sleep_mode = false;
    wake_motion_hits = 0;
    sleep_ref_initialized = false;
    sleep_wake_delta_mg = 0;

    gravity_initialized = false;

    step_counter_start_ms = k_uptime_get_32();
    last_motion_time_ms = step_counter_start_ms;
    last_ble_send_ms = step_counter_start_ms;

    printk("\nSmall motion detected. Waking from sleep mode...\n");

    start_ble_advertising();
}

/* ---------- Airplane mode ---------- */

static void airplane_start_work_handler(struct k_work *work);
static void airplane_end_work_handler(struct k_work *work);

static K_WORK_DEFINE(airplane_start_work, airplane_start_work_handler);
static K_WORK_DELAYABLE_DEFINE(airplane_end_work, airplane_end_work_handler);

static void airplane_start_work_handler(struct k_work *work)
{
    uint32_t duration_ms = airplane_req_ms;

    ARG_UNUSED(work);

    /* Set flag first so the disconnect callback can't restart advertising */
    airplane_mode = true;
    k_work_reschedule(&airplane_end_work, K_MSEC(duration_ms));

    printk("\nAirplane mode ON for %lu s. BLE going off.\n",
           (unsigned long)(duration_ms / 1000U));

    save_step_count_to_flash();

    if (current_conn && notify_enabled) {
        ble_send_text("APM:OK\n");
        k_sleep(K_MSEC(200));
    }

    if (current_conn) {
        bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    } else {
        stop_ble_advertising();
    }
}

static void airplane_end_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    airplane_mode = false;
    printk("\nAirplane mode OFF. BLE back on.\n");

    /* If in motion-sleep, advertising resumes on wake instead */
    if (!sleep_mode) {
        start_ble_advertising();
    }
}

static int airplane_request_ms(uint32_t ms)
{
    if (ms == 0 || ms > (AIRPLANE_MAX_MINUTES * 60000U)) {
        return -EINVAL;
    }

    airplane_req_ms = ms;
    k_work_submit(&airplane_start_work);
    return 0;
}

static int parse_u32(const char *s, uint32_t *out)
{
    char *end;
    unsigned long v = strtoul(s, &end, 10);

    if (end == s) {
        return -EINVAL;
    }
    while (*end == '\n' || *end == '\r' || *end == ' ') {
        end++;
    }
    if (*end != '\0') {
        return -EINVAL;
    }

    *out = (uint32_t)v;
    return 0;
}

/* ---------- BLE callbacks ---------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connection failed, err: %u\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    notify_enabled = false;
    advertising_active = false;

    printk("Mobile connected\n");
    printk("Enable TX Notify in nRF Connect Mobile\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
   printk("Mobile disconnected, reason: %u\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    notify_enabled = false;
    k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void nus_received_cb(struct bt_conn *conn,
                            const uint8_t *const data,
                            uint16_t len)
{
    char rx_buf[32];
    uint16_t copy_len = len;

    ARG_UNUSED(conn);

    if (copy_len >= sizeof(rx_buf)) {
        copy_len = sizeof(rx_buf) - 1;
    }

    memcpy(rx_buf, data, copy_len);
    rx_buf[copy_len] = '\0';

    printk("Mobile sent: %s\n", rx_buf);

    if (rx_buf[0] == 'r' || rx_buf[0] == 'R') {
        reset_step_counter(true);
    } else if (strncmp(rx_buf, "AP:", 3) == 0) {
        uint32_t minutes;

        if (parse_u32(&rx_buf[3], &minutes) == 0 &&
            minutes <= AIRPLANE_MAX_MINUTES &&
            airplane_request_ms(minutes * 60000U) == 0) {
            printk("Airplane request: %lu min\n", (unsigned long)minutes);
        } else {
            printk("Bad airplane command: %s\n", rx_buf);
        }
    } else if (strncmp(rx_buf, "APS:", 4) == 0) {
        /* Seconds variant, for quick testing only */
        uint32_t seconds;

        if (parse_u32(&rx_buf[4], &seconds) == 0 &&
            seconds <= 86400U &&
            airplane_request_ms(seconds * 1000U) == 0) {
            printk("Airplane request: %lu s\n", (unsigned long)seconds);
        } else {
            printk("Bad airplane command: %s\n", rx_buf);
        }
    }
}

static void nus_send_enabled_cb(enum bt_nus_send_status status)
{
    notify_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);

    if (notify_enabled) {
        printk("TX notifications enabled. Step count will be sent to mobile.\n");
    } else {
        printk("TX notifications disabled.\n");
    }
}

static struct bt_nus_cb nus_cb = {
    .received = nus_received_cb,
    .send_enabled = nus_send_enabled_cb,
};

/* ---------- Main ---------- */

int main(void)
{

    uint32_t reset_reason = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
    printk("Reset reason: 0x%08X\n", reset_reason);
    k_msleep(100);
    printk("BOOT\n");
    int err;

    printk("\nStarting nRF7002 DK BLE Step Counter with BMI270 Hardware Step Counter\n");

    atomic_set(&step_count, 0);
    init_runtime_state_only();

    err = step_storage_init();
    if (err) {
        printk("Step storage not available. Steps will not persist. Error: %d\n", err);
    }

    k_msleep(200);

    err = bmi270_bosch_init_sensor();
    if (err) {
        printk("BMI270 Bosch init failed: %d\n", err);
    }

    k_work_init(&adv_work, adv_work_handler);

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed: %d\n", err);
        return 0;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        printk("NUS init failed: %d\n", err);
        return 0;
    }

    start_ble_advertising();

    while (1) {
        int32_t ax, ay, az;
        int32_t motion;
        uint32_t now_ms;

        err = bmi270_read_values(&ax, &ay, &az);

        if (err) {
            printk("BMI270 read failed: %d. Reinitializing sensor...\n", err);
            bmi270_bosch_init_sensor();
            gravity_initialized = false;
            sleep_ref_initialized = false;
            k_sleep(K_MSEC(1000));
            continue;
        }

        if (!bmi270_accel_is_valid(ax, ay, az)) {
            printk("BMI270 invalid accel 0,0,0. Reinitializing sensor...\n");
            bmi270_bosch_init_sensor();
            gravity_initialized = false;
            sleep_ref_initialized = false;
            k_sleep(K_MSEC(500));
            continue;
        }

        motion = calculate_motion_mg(ax, ay, az);
        now_ms = k_uptime_get_32();

        if (sleep_mode) {
            if (small_motion_detected_in_sleep(ax, ay, az)) {
                wake_motion_hits++;
            } else {
                wake_motion_hits = 0;
            }

            printk("SLEEP | AX:%ld AY:%ld AZ:%ld | WakeDelta:%ldmg | Hits:%u\n",
                   (long)ax,
                   (long)ay,
                   (long)az,
                   (long)sleep_wake_delta_mg,
                   wake_motion_hits);

            if (wake_motion_hits >= WAKE_HIT_COUNT_REQUIRED) {
                wake_from_sleep_mode();
            }

            k_sleep(K_MSEC(SLEEP_SAMPLE_INTERVAL_MS));
            continue;
        }

        if (motion > NO_MOTION_THRESHOLD_MG) {
            last_motion_time_ms = now_ms;
        }

        /*
         * Step count is updated only from BMI270 built-in step counter.
         * Raw motion is NOT used to increment steps.
         */
        bmi270_update_step_count_from_hw();
        save_step_count_if_needed();

        printk("AX:%ld AY:%ld AZ:%ld | Motion:%ldmg | HW:%lu | Steps:%ld | Saved:%lu | Idle:%lds\n",
               (long)ax,
               (long)ay,
               (long)az,
               (long)motion,
               (unsigned long)bmi270_hw_steps_last,
               (long)atomic_get(&step_count),
               (unsigned long)last_saved_step_count,
               (long)((now_ms - last_motion_time_ms) / 1000));

        if ((uint32_t)(now_ms - last_ble_send_ms) >= BLE_SEND_INTERVAL_MS) {
            last_ble_send_ms = now_ms;
            send_step_count_to_mobile();
        }

        if ((uint32_t)(now_ms - last_motion_time_ms) >= NO_MOTION_TIMEOUT_MS) {
            enter_sleep_mode();
        }

        k_sleep(K_MSEC(SENSOR_SAMPLE_INTERVAL_MS));
    }

    return 0;
}