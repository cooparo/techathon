/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reflector (Beacon) - Modified
 * 1. Advertises immediately on boot.
 * 2. Accepts any connection.
 * 3. LED0 ON when connected, OFF when disconnected/advertising.
 * 4. All button logic is removed.
 */

#include <math.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h> 
#include "common.h"

#define CS_CONFIG_ID 0
#define NUM_MODE_0_STEPS 1

/* --- MODIFIED: GPIO setup --- */
#define LED0_PORT DT_NODELABEL(gpio2)
#define LED0_PIN 9
/* Button and LED3 removed */

static const struct device *led0_dev;
/* Button device and callback data removed */

/* --- Bluetooth stuff --- */
static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_done, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovered, 0, 1);
static K_SEM_DEFINE(sem_written, 0, 1);

static uint16_t step_data_attr_handle;
static struct bt_conn *connection;
static uint8_t latest_local_steps[STEP_DATA_BUF_LEN];

static const char sample_str[] = "Beacon";
static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "beacon", sizeof(sample_str) - 1),
};

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
    if (result->step_data_buf) {
        if (result->step_data_buf->len <= STEP_DATA_BUF_LEN) {
            memcpy(latest_local_steps, result->step_data_buf->data,
                   result->step_data_buf->len);
        } else {
            printk("Not enough memory to store step data. (%d > %d)\n",
                   result->step_data_buf->len, STEP_DATA_BUF_LEN);
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
    printk("Connected to %s (err 0x%02X)\n", addr, err);

    /* Check for connection object consistency */
    if (connection) {
         printk("Warning: Already connected\n");
         return; 
    }

    if (err) {
        bt_conn_unref(conn);
        connection = NULL;
        /* Ensure LED is off if connection fails */
        gpio_pin_set(led0_dev, LED0_PIN, 0);
        return;
    }

    connection = bt_conn_ref(conn);

    /* Turn LED0 ON (solid) if connected */
    gpio_pin_set(led0_dev, LED0_PIN, 1);

    static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

    err = bt_gatt_exchange_mtu(connection, &mtu_exchange_params);
    if (err) {
        printk("%s: MTU exchange failed (err %d)\n", __func__, err);
    }

    k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02X)\n", reason);

    bt_conn_unref(conn);
    connection = NULL;
    /* Turn LED0 OFF */
    gpio_pin_set(led0_dev, LED0_PIN, 0);
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

/* --- GATT discovery and write --- */
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    struct bt_gatt_chrc *chrc;
    char str[BT_UUID_STR_LEN];

    printk("Discovery: attr %p\n", attr);

    if (!attr) {
        return BT_GATT_ITER_STOP;
    }

    chrc = (struct bt_gatt_chrc *)attr->user_data;

    bt_uuid_to_str(chrc->uuid, str, sizeof(str));
    printk("UUID %s\n", str);

    if (!bt_uuid_cmp(chrc->uuid, &step_data_char_uuid.uuid)) {
        step_data_attr_handle = chrc->value_handle;

        printk("Found expected UUID\n");

        k_sem_give(&sem_discovered);
    }

    return BT_GATT_ITER_STOP;
}

static void write_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    if (err) {
        printk("Write failed (err %d)\n", err);

        return;
    }

    k_sem_give(&sem_written);
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
    .le_cs_config_complete = config_create_cb,
    .le_cs_security_enable_complete = security_enable_cb,
    .le_cs_procedure_enable_complete = procedure_enable_cb,
    .le_cs_subevent_data_available = subevent_result_cb,
};

/* GPIO init */
static void gpio_init(void)
{
    led0_dev = DEVICE_DT_GET(LED0_PORT);
    
    if (!device_is_ready(led0_dev)) {
        printk("Error: GPIO device(s) not ready\n");
        return;
    }

    gpio_pin_configure(led0_dev, LED0_PIN, GPIO_OUTPUT_INACTIVE);

}

int main(void)
{
    int err;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_write_params write_params;

    printk("Starting Reflector (Beacon/Totem) Demo\n");

    gpio_init();

    bt_unpair(BT_ID_DEFAULT, NULL);

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    while (true) {
        /* Start advertising immediately */
        printk("Advertising enabled, waiting for connection...\n");

        /* Ensure LED0 is OFF while advertising */
        gpio_pin_set(led0_dev, LED0_PIN, 0);

        err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("Advertising failed (err %d)\n", err);
            gpio_pin_set(led0_dev, LED0_PIN, 0);
            k_sleep(K_SECONDS(1));
            continue;
        }

        /* Wait for connection */
        /* connected_cb will set LED0 ON */
        k_sem_take(&sem_connected, K_FOREVER);

        /* Stop advertising once connected */
        bt_le_adv_stop();

        /* Proceed with CS and GATT logic */
        const struct bt_le_cs_set_default_settings_param default_settings = {
            .enable_initiator_role = false,
            .enable_reflector_role = true,
            .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
            .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
        };

        err = bt_le_cs_set_default_settings(connection, &default_settings);
        if (err) {
            printk("Failed to configure default CS settings (err %d)\n", err);
        }

        discover_params.uuid = &step_data_char_uuid.uuid;
        discover_params.func = discover_func;
        discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
        discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(connection, &discover_params);
        if (err) {
            printk("Discovery failed (err %d)\n", err);
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }
        err = k_sem_take(&sem_discovered, K_SECONDS(10));
        if (err) {
            printk("Timed out during GATT discovery\n");
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            continue;
        }

        /* Loop while connected */
        while (connection) {
            if (k_sem_take(&sem_procedure_done, K_MSEC(1000)) != 0) {
                /* Check if still connected */
                if (connection == NULL) {
                    break;
                }
                continue;
            }

            write_params.func = write_func;
            write_params.handle = step_data_attr_handle;
            write_params.length = STEP_DATA_BUF_LEN;
            write_params.data = &latest_local_steps[0],
            write_params.offset = 0;

            err = bt_gatt_write(connection, &write_params);
            if (err) {
                printk("Write failed (err %d)\n", err);
                break;
            }

            err = k_sem_take(&sem_written, K_SECONDS(10));
            if (err) {
                printk("Timed out during GATT write\n");
                break;
            }
        }

        /* disconnected_cb will have set LED0 OFF */
        printk("Disconnected. Restarting advertising...\n");
    }

    return 0;
}