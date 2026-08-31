/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gap.h"
#include "common.h"
#include "led.h"

/* Private function declarations */
inline static void format_addr(char *addr_str, uint8_t addr[]);
static void print_conn_desc(struct ble_gap_conn_desc *desc);
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static int adv_report_handler(struct ble_gap_event *event, void *arg);

/* Private variables */
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static uint16_t conn_handle;


/* Private functions */
inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static void print_conn_desc(struct ble_gap_conn_desc *desc) {
    /* Local variables */
    char addr_str[18] = {0};

    /* Connection handle */
    ESP_LOGI(TAG, "connection handle: %d", desc->conn_handle);

    /* Local ID address */
    format_addr(addr_str, desc->our_id_addr.val);
    ESP_LOGI(TAG, "device id address: type=%d, value=%s",
             desc->our_id_addr.type, addr_str);

    /* Peer ID address */
    format_addr(addr_str, desc->peer_id_addr.val);
    ESP_LOGI(TAG, "peer id address: type=%d, value=%s", desc->peer_id_addr.type,
             addr_str);

    /* Connection info */
    ESP_LOGI(TAG,
             "conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, "
             "encrypted=%d, authenticated=%d, bonded=%d\n",
             desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
             desc->sec_state.encrypted, desc->sec_state.authenticated,
             desc->sec_state.bonded);
}

static void disconnect_from_dev(void) {
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);

    led_on(0, 16, 0); //red
    vTaskDelay(pdMS_TO_TICKS(1000));
}

static int updt_conn_params(struct ble_gap_upd_params *params) {
    /*struct ble_gap_upd_params params;

    memset(&params, 0, sizeof(params));

    params.itvl_min = 24;
    params.itvl_max = 40;

    params.latency = 0;

    params.supervision_timeout = 400;

    params.min_ce_len = 0;
    params.max_ce_len = 0;*/

    int rc = ble_gap_update_params(conn_handle, params);

    if (rc != 0) {
        ESP_LOGE(TAG, "failed to update connection parameters, error code: %d", rc);
        return rc;
    }

    return 0;
}

static void start_scanning(void) {
    struct ble_gap_disc_params disc_params;

    ESP_LOGE(TAG, "Started scanning");
    led_on(0, 0, 16); //blue

    memset(&disc_params, 0, sizeof(disc_params));

    disc_params.passive = 1;
    disc_params.itvl = 0; //scanning interval == 0 => use default val
    disc_params.window = 0; // use default val
    disc_params.filter_duplicates = 1;

    ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, adv_report_handler, NULL);
}

/**
 * @brief callback function associated with the discovery process. Adveritisng reports and 
 *        discovery termination events are handled through this function.
 */
static int adv_report_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC:
        printf("Device found\n");

        struct ble_hs_adv_fields fields;

        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

        if (rc != 0) {
            return 0;
        }

        if (fields.name != NULL && fields.name_len == strlen(TARGET_NAME) 
            && memcmp(fields.name, TARGET_NAME, fields.name_len) == 0) {
            printf("Found our Peripheral!\n");
            ble_gap_disc_cancel(); //stop discovery
            ESP_LOGE(TAG, "Started scanning");

            //set connection params
           rc =  ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, gap_event_handler, NULL);
            
           if (rc != 0) {
            printf("Connection unsuccessful, reason: %d\n", rc);
            start_scanning();
            return 0;
           }
        }

    break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        printf("Scan complete\n");
    break;
    }

    return 0;
}

/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    /* Local variables */
    int rc = 0;
    struct ble_gap_conn_desc desc;

    /* Handle different GAP event */
    switch (event->type) {

    /* Connect event */
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        /* Connection succeeded */
        if (event->connect.status == 0) {
            /* Check connection handle */
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(TAG,
                         "failed to find connection by handle, error code: %d",
                         rc);
                return rc;
            }

            conn_handle = event->connect.conn_handle;

            /* Print connection descriptor and turn on the LED */
            print_conn_desc(&desc);
            led_on(16, 0, 0); //green

            /* Try to update connection parameters */
            struct ble_gap_upd_params params = {.itvl_min = 24,
                                                .itvl_max = 40,
                                                .latency = 3,
                                                .supervision_timeout =
                                                    desc.supervision_timeout};
        
            return updt_conn_params(&params);
        }
        /* Connection failed, restart advertising */
        else {
            start_scanning();
        }
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(TAG, "disconnected from peer; reason=%d",
                 event->disconnect.reason);

        disconnect_from_dev();

        /* Turn off the LED */
        led_off();
        ESP_LOGE(TAG, "LED OFF");

        /* Restart advertising */
        start_scanning();
        return rc;

    /* Connection parameters update event */
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The peripheral device has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d",
                 event->conn_update.status);

        /* Print connection descriptor */
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(TAG, "failed to find connection by handle, error code: %d",
                     rc);
            return rc;
        }
        print_conn_desc(&desc);
        return rc;
    }

    return rc;
}

/* Public functions */
void scan_init(void) {
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(0); //gets a public addr if it is available and if not gets a random addr
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type); //returns either BLE_OWN_ADDR_(RANDOM/PUBLIC)
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL); // gets the address ble_hs_id_pub
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);

    start_scanning();
}

int gap_init(void) {
    /* Local variables */
    int rc = 0;


    /* Initialize GAP service */
    ble_svc_gap_init();

    /* Set GAP device name */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d",
                 DEVICE_NAME, rc);
        return rc;
    }

    /* Set GAP device appearance */
    rc = ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE_GENERIC_TAG);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device appearance, error code: %d", rc);
        return rc;
    }
    return rc;
}
