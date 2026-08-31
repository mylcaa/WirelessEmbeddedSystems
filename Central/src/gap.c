/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gap.h"
#include "common.h"
#include "led.h"

/*************************************************************************************/
/*                          PRIVATE DEFINITIONS AND TYPES                            */
/*************************************************************************************/

/* Private function declarations */
inline static void format_addr(char *addr_str, uint8_t addr[]);
static int parse_addr_str(const char *addr_str, ble_addr_t *addr);
static void print_conn_desc(struct ble_gap_conn_desc *desc);
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static int scan_event_handler(struct ble_gap_event *event, void *arg);

/* Private variables */
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static uint16_t conn_handle;
static gap_state_t gap_state = GAP_STATE_IDLE;

static ble_addr_t connected_addr;
static char connected_name[32] = {0};

typedef struct {
    ble_addr_t addr;
    char name[32];
} discovered_dev_t;

static discovered_dev_t discovered_devices[MAX_DISCOVERED_DEVICES];
static int num_discovered = 0;

/*************************************************************************************/
/*                                HELPER FUNCTIONS                                   */
/*************************************************************************************/

/* Private functions */
inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static int parse_addr_str(const char *addr_str, ble_addr_t *addr) {
    int bytes[6] = {0};
    if (sscanf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4],
               &bytes[5]) != 6) {
        return -1;
    }
    for (int i = 0; i < 6; i++) {
        addr->val[i] = (uint8_t)bytes[i];
    }
    return 0;
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

static int updt_conn_params(struct ble_gap_upd_params *params) {
    int rc = ble_gap_update_params(conn_handle, params);

    if (rc != 0) {
        ESP_LOGE(TAG, "failed to update connection parameters, error code: %d", rc);
        return rc;
    }

    return 0;
}

/*************************************************************************************/
/*                               CALLBACK FUNCTIONS                                  */
/*************************************************************************************/

static int scan_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        char addr_str[18] = {0};

        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        format_addr(addr_str, event->disc.addr.val);

        char name_buf[32] = {0};
        if (fields.name != NULL && fields.name_len > 0) {
            int len = fields.name_len < sizeof(name_buf) - 1 ? fields.name_len
                                                             : sizeof(name_buf) - 1;
            memcpy(name_buf, fields.name, len);
            name_buf[len] = '\0';
        }

        ESP_LOGI(TAG, "Found device addr=%s type=%d name=%s", addr_str,
                 event->disc.addr.type,
                 name_buf[0] != '\0' ? name_buf : "(unknown)");

        /* Store discovered device */
        if (num_discovered < MAX_DISCOVERED_DEVICES) {
            discovered_devices[num_discovered].addr = event->disc.addr;
            strncpy(discovered_devices[num_discovered].name, name_buf,
                    sizeof(discovered_devices[num_discovered].name) - 1);
            discovered_devices[num_discovered].name[
                sizeof(discovered_devices[num_discovered].name) - 1] = '\0';
            num_discovered++;
        }
        break;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete");
        if (gap_state == GAP_STATE_SCANNING) {
            gap_state = GAP_STATE_IDLE;
        }
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
                gap_state = GAP_STATE_IDLE;
                return rc;
            }

            conn_handle = event->connect.conn_handle;
            connected_addr = desc.peer_id_addr;

            /* Print connection descriptor and turn on the LED */
            print_conn_desc(&desc);
            led_on(16, 0, 0); //green

            gap_state = GAP_STATE_CONNECTED;

            /* Try to update connection parameters */
            struct ble_gap_upd_params params = {.itvl_min = 24,
                                                .itvl_max = 40,
                                                .latency = 3,
                                                .supervision_timeout =
                                                    desc.supervision_timeout};

            return updt_conn_params(&params);
        }
        /* Connection failed */
        else {
            gap_state = GAP_STATE_IDLE;
        }
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(TAG, "disconnected from peer; reason=%d",
                 event->disconnect.reason);

        /* Turn off the LED */
        led_off();
        ESP_LOGE(TAG, "LED OFF");

        memset(&connected_addr, 0, sizeof(connected_addr));
        connected_name[0] = '\0';
        gap_state = GAP_STATE_IDLE;
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

/*************************************************************************************/
/*                                  INIT FUNCTIONS                                   */
/*************************************************************************************/

/* Public functions */
void device_init(void) {
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

/*************************************************************************************/
/*                               CONSOLE FUNCTIONS                                   */
/*************************************************************************************/

gap_state_t gap_get_state(void) {
    return gap_state;
}

const char *gap_state_str(void) {
    switch (gap_state) {
    case GAP_STATE_IDLE:
        return "idle";
    case GAP_STATE_SCANNING:
        return "scanning";
    case GAP_STATE_CONNECTING:
        return "connecting";
    case GAP_STATE_CONNECTED:
        return "connected";
    default:
        return "unknown";
    }
}

void gap_get_connected_info(char *addr_str, size_t addr_len, char *name, size_t name_len) {
    if (gap_state != GAP_STATE_CONNECTED) {
        if (addr_str && addr_len > 0) addr_str[0] = '\0';
        if (name && name_len > 0) name[0] = '\0';
        return;
    }

    if (addr_str && addr_len > 0) {
        format_addr(addr_str, connected_addr.val);
    }
    if (name && name_len > 0) {
        strncpy(name, connected_name, name_len - 1);
        name[name_len - 1] = '\0';
    }
}

int gap_scan_start(void) {
    if (gap_state == GAP_STATE_SCANNING) {
        ESP_LOGW(TAG, "already scanning");
        return 0;
    }
    if (gap_state == GAP_STATE_CONNECTED || gap_state == GAP_STATE_CONNECTING) {
        ESP_LOGE(TAG, "cannot scan while connecting/connected");
        return -1;
    }

    struct ble_gap_disc_params disc_params;

    ESP_LOGI(TAG, "Started scanning");
    led_on(0, 0, 16); //blue

    memset(&disc_params, 0, sizeof(disc_params));

    disc_params.passive = 1;
    disc_params.itvl = 0; //scanning interval == 0 => use default val
    disc_params.window = 0; // use default val
    disc_params.filter_duplicates = 1;

    /* Clear previous discovery cache */
    num_discovered = 0;
    memset(discovered_devices, 0, sizeof(discovered_devices));

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                          scan_event_handler, NULL);
    if (rc == 0) {
        gap_state = GAP_STATE_SCANNING;
    } else {
        ESP_LOGE(TAG, "failed to start scanning, error code: %d", rc);
        led_off();
        gap_state = GAP_STATE_IDLE;
    }

    return rc;
}

int gap_scan_stop(void) {
    if (gap_state != GAP_STATE_SCANNING) {
        ESP_LOGW(TAG, "not scanning");
        return 0;
    }

    int rc = ble_gap_disc_cancel();
    if (rc == 0) {
        gap_state = GAP_STATE_IDLE;
        led_off();
        ESP_LOGI(TAG, "Scanning stopped");
    } else {
        ESP_LOGE(TAG, "failed to stop scanning, error code: %d", rc);
    }
    return rc;
}

int gap_connect_addr_str(const char *addr_str) {
    if (gap_state == GAP_STATE_CONNECTED || gap_state == GAP_STATE_CONNECTING) {
        ESP_LOGE(TAG, "already connected or connecting");
        return -1;
    }

    /* Look up the address in the discovered cache */
    for (int i = 0; i < num_discovered; i++) {
        char found_addr[18] = {0};
        format_addr(found_addr, discovered_devices[i].addr.val);
        if (strcasecmp(found_addr, addr_str) == 0) {
            ESP_LOGI(TAG, "Connecting to %s (%s)", addr_str,
                     discovered_devices[i].name[0] != '\0'
                         ? discovered_devices[i].name
                         : "(unknown)");

            // Have to stop discovery process before attempting to connect
            ble_gap_disc_cancel();

            gap_state = GAP_STATE_CONNECTING;
            int rc = ble_gap_connect(own_addr_type, &discovered_devices[i].addr,
                                     30000, NULL, gap_event_handler, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Connection unsuccessful, reason: %d", rc);
                return rc;
            }
            strncpy(connected_name, discovered_devices[i].name,
                    sizeof(connected_name) - 1);
            connected_name[sizeof(connected_name) - 1] = '\0';
            return 0;
        }
    }

    ESP_LOGE(TAG, "Connection unsuccessful, wrong addr");
    connected_name[0] = '\0';
    return 0;
}

int gap_disconnect_by_addr_or_name(const char *addr_or_name) {
    if (gap_state != GAP_STATE_CONNECTED) {
        ESP_LOGE(TAG, "not connected");
        return -1;
    }

    char addr_str[18] = {0};
    format_addr(addr_str, connected_addr.val);

    if (strcasecmp(addr_or_name, addr_str) != 0 &&
        strcasecmp(addr_or_name, connected_name) != 0) {
        ESP_LOGE(TAG, "not connected to %s", addr_or_name);
        return -1;
    }

    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to disconnect, error code: %d", rc);
        return rc;
    }

    gap_state = GAP_STATE_IDLE;
    memset(&connected_addr, 0, sizeof(connected_addr));
    connected_name[0] = '\0';
    ESP_LOGI(TAG, "Disconnected from %s", addr_or_name);
    return 0;
}
