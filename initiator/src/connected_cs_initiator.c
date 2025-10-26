/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Initiator (Rune) - Modified
 * 1. Scans for Reflector ("Beacon").
 * 2. On Find: Start BLINKING LED0, wait for button press (with timeout).
 * 3. On Button Press: Turn OFF LED0, connect and start CS.
 * 4. On Timeout: Turn OFF LED0, restart scan.
 */

#include <math.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h> 
#include <zephyr/toolchain.h> 
#include "distance_estimation.h"
#include "common.h"

#define CS_CONFIG_ID     0
#define NUM_MODE_0_STEPS 1

/* --- GPIO setup --- */
#define LED0_PORT DT_NODELABEL(gpio2)
#define LED0_PIN 9

#define BUTTON0_PORT DT_NODELABEL(gpio1)
#define BUTTON0_PIN 13
#define BUTTON_PRESS_TIMEOUT_S 10

static const struct device *led0_dev, *btn0_dev;
static struct gpio_callback btn_cb_data;

static char connected_reflector_name[NAME_LEN] = "Reflector";

static K_SEM_DEFINE(sem_acl_encryption_enabled, 0, 1);
static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_done, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_data_received, 0, 1);
static K_SEM_DEFINE(sem_button_pressed, 0, 1);

static struct k_timer led_blink_timer;
static volatile bool led_is_on = false;

static struct bt_conn *connection;
static uint8_t n_ap;
static uint8_t latest_num_steps_reported;
static uint16_t latest_step_data_len;
static uint8_t latest_local_steps[STEP_DATA_BUF_LEN];
static uint8_t latest_peer_steps[STEP_DATA_BUF_LEN];

static bt_addr_le_t found_addr;
static volatile bool awaiting_connect_confirmation = false;

/* Scan interval: 0x0020 = 32 * 0.625ms = 20ms */
/* Scan window:   0x0020 = 32 * 0.625ms = 20ms */
static struct bt_le_scan_param fast_scan_params = {
    .type = BT_LE_SCAN_TYPE_ACTIVE,
    .options = BT_LE_SCAN_OPT_NONE,
    .interval = 0x0020,
    .window = 0x0020,
};

/* Time handler for blinking LED */
static void blink_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    led_is_on = !led_is_on;
    gpio_pin_set(led0_dev, LED0_PIN, (int)led_is_on);
}

static ssize_t on_attr_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
        return 0;
    }

    if (offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(latest_local_steps)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (flags & BT_GATT_WRITE_FLAG_EXECUTE) {
        uint8_t *data = (uint8_t *)buf;

        memcpy(latest_peer_steps, &data[offset], len);
        k_sem_give(&sem_data_received);
    }

    return len;
}

static struct bt_gatt_attr gatt_attributes[] = {
    BT_GATT_PRIMARY_SERVICE(&step_data_svc_uuid),
    BT_GATT_CHARACTERISTIC(&step_data_char_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE | BT_GATT_PERM_PREPARE_WRITE, NULL,
                           on_attr_write_cb, NULL),
};
static struct bt_gatt_service step_data_gatt_service = BT_GATT_SERVICE(gatt_attributes);
static const char sample_str[] = "Beacon";

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
    latest_num_steps_reported = result->header.num_steps_reported;
    n_ap = result->header.num_antenna_paths;

    if (result->step_data_buf) {
        if (result->step_data_buf->len <= STEP_DATA_BUF_LEN) {
            memcpy(latest_local_steps, result->step_data_buf->data,
                   result->step_data_buf->len);
            latest_step_data_len = result->step_data_buf->len;
        } else {
            printk("Not enough memory to store step data. (%d > %d)\n",
                   result->step_data_buf->len, STEP_DATA_BUF_LEN);
            latest_num_steps_reported = 0;
        }
    }

    if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
        k_sem_give(&sem_procedure_done);
    }
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    printk("MTU exchange %s (%u)\n", err == 0U ? "success" : "failed", bt_gatt_get_mtu(conn));
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    (void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        printk("Failed to connect to %s (err 0x%02X)\n", addr, err);
        bt_conn_unref(conn);
        connection = NULL;
    } else {
        printk("Connected to %s (err 0x%02X)\n", addr, err);
        connection = bt_conn_ref(conn);

        static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

        err = bt_gatt_exchange_mtu(connection, &mtu_exchange_params);
        if (err) {
            printk("%s: MTU exchange failed (err %d)\n", __func__, err);
        }
    }

    k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02X)\n", reason);

    bt_conn_unref(conn);
    connection = NULL;
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    if (err) {
        printk("Encryption failed. (err %d)\n", err);
    } else {
        printk("Security changed to level %d.\n", level);
    }

    k_sem_give(&sem_acl_encryption_enabled);
}

static void remote_capabilities_cb(struct bt_conn *conn,
                                   uint8_t status,
                                   struct bt_conn_le_cs_capabilities *params)
{
    ARG_UNUSED(params);

    if (status == BT_HCI_ERR_SUCCESS) {
        printk("CS capability exchange completed.\n");
        k_sem_give(&sem_remote_capabilities_obtained);
    } else {
        printk("CS capability exchange failed. (HCI status 0x%02x)\n", status);
    }
}

static void config_create_cb(struct bt_conn *conn,
                             uint8_t status,
                             struct bt_conn_le_cs_config *config)
{
    if (status == BT_HCI_ERR_SUCCESS) {
        printk("CS config creation complete. ID: %d\n", config->id);
        k_sem_give(&sem_config_created);
    } else {
        printk("CS config creation failed. (HCI status 0x%02x)\n", status);
    }
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
    if (status == BT_HCI_ERR_SUCCESS) {
        printk("CS security enabled.\n");
        k_sem_give(&sem_cs_security_enabled);
    } else {
        printk("CS security enable failed. (HCI status 0x%02x)\n", status);
    }
}

static void procedure_enable_cb(struct bt_conn *conn,
                                uint8_t status,
                                struct bt_conn_le_cs_procedure_enable_complete *params)
{
    if (status == BT_HCI_ERR_SUCCESS) {
        if (params->state == 1) {
            printk("CS procedures enabled.\n");
        } else {
            printk("CS procedures disabled.\n");
        }
    } else {
        printk("CS procedures enable failed. (HCI status 0x%02x)\n", status);
    }
}

static bool data_cb(struct bt_data *data, void *user_data)
{
    char *name = user_data;
    uint8_t len;

    switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        len = MIN(data->data_len, NAME_LEN - 1);
        memcpy(name, data->data, len);
        name[len] = '\0';
        return false;
    default:
        return true;
    }
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    char name[NAME_LEN] = {};
    if (connection || awaiting_connect_confirmation) {
        return;
    }

    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    bt_data_parse(ad, data_cb, name);

    if (strncasecmp(name, sample_str, sizeof(sample_str) - 1) != 0) {
        return;
    }

    if (bt_le_scan_stop()) {
        printk("Failed to stop scan\n");
        return;
    }

    printk("Found Totem %s. Press Button 0 to connect...\n", name);

    bt_addr_le_copy(&found_addr, addr);
    memcpy(connected_reflector_name, name, NAME_LEN);
    awaiting_connect_confirmation = true;

    led_is_on = true;
    gpio_pin_set(led0_dev, LED0_PIN, 1);
    /* Blink every 250ms */
    k_timer_start(&led_blink_timer, K_MSEC(250), K_MSEC(250));
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (awaiting_connect_confirmation) {
        k_sem_give(&sem_button_pressed);
    }
}

static void gpio_init(void)
{
    led0_dev = DEVICE_DT_GET(LED0_PORT);
    btn0_dev = DEVICE_DT_GET(BUTTON0_PORT);

    if (!device_is_ready(led0_dev) || !device_is_ready(btn0_dev)) {
        printk("Error: GPIO device(s) not ready\n");
        return;
    }

    gpio_pin_configure(led0_dev, LED0_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(btn0_dev, BUTTON0_PIN, GPIO_INPUT | GPIO_PULL_UP);

    gpio_pin_interrupt_configure(btn0_dev, BUTTON0_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb_data, button_pressed, BIT(BUTTON0_PIN));
    gpio_add_callback(btn0_dev, &btn_cb_data);
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .security_changed = security_changed_cb,
    .le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
    .le_cs_config_complete = config_create_cb,
    .le_cs_security_enable_complete = security_enable_cb,
    .le_cs_procedure_enable_complete = procedure_enable_cb,
    .le_cs_subevent_data_available = subevent_result_cb,
};

int main(void)
{
    int err;

    printk("Starting Channel Sounding Demo (Initiator/Rune)\n");

    gpio_init();

    /* Initialize blink timer */
    k_timer_init(&led_blink_timer, blink_timer_handler, NULL);
    
    /* Clear all old bonds to prevent security errors */
    bt_unpair(BT_ID_DEFAULT, NULL);

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    err = bt_gatt_service_register(&step_data_gatt_service);
    if (err) {
        printk("bt_gatt_service_register() returned err %d\n", err);
        return 0;
    }

    while (true) {
        printk("Starting scan for Totem...\n");
        awaiting_connect_confirmation = false;
        led_is_on = false;
        gpio_pin_set(led0_dev, LED0_PIN, 0);
        
        k_sem_reset(&sem_button_pressed);
        k_sem_reset(&sem_connected);

        err = bt_le_scan_start(&fast_scan_params, device_found);
        if (err) {
            printk("Scanning failed to start (err %d)\n", err);
            return 0;
        }

        /* Wait until device_found() (ISR) runs and sets the flag */
        while (!awaiting_connect_confirmation) {
            k_msleep(100);
        }
        /* At this point, scan is stopped, LED0 is BLINKING, and found_addr is valid */

        /* Wait for button_pressed() (ISR), with a timeout */
        int button_press_result = k_sem_take(&sem_button_pressed, K_SECONDS(BUTTON_PRESS_TIMEOUT_S));

        k_timer_stop(&led_blink_timer);
        led_is_on = false;
        gpio_pin_set(led0_dev, LED0_PIN, 0);

        if (button_press_result != 0) {
            printk("Button press timed out (%ds). Restarting scan.\n", BUTTON_PRESS_TIMEOUT_S);
            awaiting_connect_confirmation = false;
            continue;
        }
        
        /* Button was pressed and connects */
        printk("Button pressed, attempting to connect...\n");
        awaiting_connect_confirmation = false;

        err = bt_conn_le_create(&found_addr, BT_CONN_LE_CREATE_CONN,
                                BT_LE_CONN_PARAM_DEFAULT, &connection);
        if (err) {
            printk("Create conn failed (%u). Restarting scan.\n", err);
            connection = NULL; 
            continue;
        }

        k_sem_take(&sem_connected, K_FOREVER);

        /* Check if connection was successful */
        if (connection == NULL) {
            printk("Connection attempt failed, restarting scan.\n");
            continue;
        }

        printk("Connection successful. Proceeding with CS setup.\n");

        
        const struct bt_le_cs_set_default_settings_param default_settings = {
            .enable_initiator_role = true,
            .enable_reflector_role = false,
            .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
            .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
        };

        err = bt_le_cs_set_default_settings(connection, &default_settings);
        if (err) {
            printk("Failed to configure default CS settings (err %d)\n", err);
        }

        err = bt_conn_set_security(connection, BT_SECURITY_L2);
        if (err) {
            printk("Failed to encrypt connection (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }

        k_sem_take(&sem_acl_encryption_enabled, K_FOREVER);
        if (connection == NULL) { continue; }


        err = bt_le_cs_read_remote_supported_capabilities(connection);
        if (err) {
            printk("Failed to exchange CS capabilities (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }

        k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);
        if (connection == NULL) { continue; }

        struct bt_le_cs_create_config_params config_params = {
            .id = CS_CONFIG_ID,
            .main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
            .sub_mode_type = BT_CONN_LE_CS_SUB_MODE_1,
            .min_main_mode_steps = 2,
            .max_main_mode_steps = 4,
            .main_mode_repetition = 0,
            .mode_0_steps = NUM_MODE_0_STEPS,
            .role = BT_CONN_LE_CS_ROLE_INITIATOR,
            .rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
            .cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
            .channel_map_repetition = 1,
            .channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
            .ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
            .ch3c_jump = 2,
        };

        bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

        err = bt_le_cs_create_config(connection, &config_params,
                                   BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
        if (err) {
            printk("Failed to create CS config (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }

        k_sem_take(&sem_config_created, K_FOREVER);
        if (connection == NULL) { continue; }

        err = bt_le_cs_security_enable(connection);
        if (err) {
            printk("Failed to start CS Security (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }

        k_sem_take(&sem_cs_security_enabled, K_FOREVER);
        if (connection == NULL) { continue; }

        const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
            .config_id = CS_CONFIG_ID,
            .max_procedure_len = 12,
            .min_procedure_interval = 100,
            .max_procedure_interval = 100,
            .max_procedure_count = 0,
            .min_subevent_len = 6750,
            .max_subevent_len = 6750,
            .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
            .phy = BT_LE_CS_PROCEDURE_PHY_1M,
            .tx_power_delta = 0x80,
            .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
            .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
            .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
        };

        err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
        if (err) {
            printk("Failed to set procedure parameters (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }

        struct bt_le_cs_procedure_enable_param params = {
            .config_id = CS_CONFIG_ID,
            .enable = 1,
        };

        err = bt_le_cs_procedure_enable(connection, &params);
        if (err) {
            printk("Failed to enable CS procedures (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }
        
        while (connection != NULL) {
            if (k_sem_take(&sem_procedure_done, K_MSEC(1000)) != 0) {
                 if (connection == NULL) { break; }
                 continue;
            }
            if (k_sem_take(&sem_data_received, K_MSEC(1000)) != 0) {
                 if (connection == NULL) { break; }
                 continue;
            }

            estimate_distance(
                latest_local_steps, latest_step_data_len, latest_peer_steps,
                latest_step_data_len -
                NUM_MODE_0_STEPS *
                    (sizeof(struct bt_hci_le_cs_step_data_mode_0_initiator) -
                     sizeof(struct bt_hci_le_cs_step_data_mode_0_reflector)),
                n_ap, BT_CONN_LE_CS_ROLE_INITIATOR,
                connected_reflector_name);
        }

        printk("Disconnected. Looping back to scan.\n");
    }

    return 0;
}