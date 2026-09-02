/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/
#include "adapt_ble_gatt.h"
#include "adapt_ble.h"
#include "common.h"
#include "gap.h"

#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "esp_bt.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef ADAPT_BLE

/*************************************************************************************/
/*                               DEFINITIONS AND TYPES                               */
/*************************************************************************************/

/* AdaptBLE service/characteristic UUIDs exposed by the slave */
#define ADAPT_BLE_TX_PWR_SVC_UUID      0xAD80
#define ADAPT_BLE_TX_PWR_CHR_UUID      0xAD81
#define ADAPT_BLE_STREAM_SVC_UUID      0xAD90
#define ADAPT_BLE_STREAM_CHR_UUID      0xAD91

/* Handle fallbacks from menuconfig */
#ifdef CONFIG_ADAPT_BLE_SLAVE_TX_PWR_HANDLE
#define ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE CONFIG_ADAPT_BLE_SLAVE_TX_PWR_HANDLE
#else
#define ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE 0x0008
#endif

#ifdef CONFIG_ADAPT_BLE_SLAVE_STREAM_HANDLE
#define ADAPT_BLE_SLAVE_STREAM_ATTR_HANDLE CONFIG_ADAPT_BLE_SLAVE_STREAM_HANDLE
#else
#define ADAPT_BLE_SLAVE_STREAM_ATTR_HANDLE 0x000B
#endif

/* Dummy stream parameters (emulates ~40 kbps of ACL traffic) */
#define STREAM_PKT_SIZE   100
#define STREAM_PERIOD_MS  40

/* GATT discovery state machine */
typedef enum {
    DISC_STATE_IDLE,
    DISC_STATE_TX_PWR_SVC,
    DISC_STATE_TX_PWR_CHR,
    DISC_STATE_STREAM_SVC,
    DISC_STATE_STREAM_CHR,
    DISC_STATE_DONE,
} disc_state_t;

static disc_state_t g_disc_state = DISC_STATE_IDLE;
static uint16_t g_disc_svc_start = 0;
static uint16_t g_disc_svc_end = 0;

/* Discovered characteristic handles; 0 means not discovered yet */
static uint16_t g_discovered_tx_pwr_handle = 0;
static uint16_t g_discovered_stream_handle = 0;

/* Dummy stream task handle */
static TaskHandle_t g_stream_task_handle = NULL;

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg);
static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg);

static void stream_gatt_start(void);

/*************************************************************************************/
/*                                  PUBLIC HELPERS                                   */
/*************************************************************************************/

uint16_t adapt_ble_gatt_get_tx_pwr_handle(void) {
    return g_discovered_tx_pwr_handle ? g_discovered_tx_pwr_handle
                                       : ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE;
}

uint16_t adapt_ble_gatt_get_stream_handle(void) {
    return g_discovered_stream_handle ? g_discovered_stream_handle
                                      : ADAPT_BLE_SLAVE_STREAM_ATTR_HANDLE;
}

/*************************************************************************************/
/*                              DISCOVERY STATE MACHINE                              */
/*************************************************************************************/

static int disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
    (void)arg;

    if (error->status == BLE_HS_EDONE) {
        if (g_disc_state == DISC_STATE_TX_PWR_CHR) {
            g_disc_state = DISC_STATE_STREAM_SVC;
            int rc = ble_gattc_disc_svc_by_uuid(conn_handle,
                                                 BLE_UUID16_DECLARE(ADAPT_BLE_STREAM_SVC_UUID),
                                                 disc_svc_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Discovery: failed to start stream service search, rc=%d", rc);
                g_disc_state = DISC_STATE_IDLE;
            }
        } else if (g_disc_state == DISC_STATE_STREAM_CHR) {
            g_disc_state = DISC_STATE_DONE;
            ESP_LOGI(TAG,
                     "Discovery complete: tx_pwr_handle=0x%04X, stream_handle=0x%04X",
                     g_discovered_tx_pwr_handle, g_discovered_stream_handle);

            if (g_discovered_stream_handle != 0) {    
                /* Start dummy stream now */
                stream_gatt_start();
            }
        }
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Discovery: characteristic search failed, status=%d", error->status);
        g_disc_state = DISC_STATE_IDLE;
        return 0;
    }

    if (g_disc_state == DISC_STATE_TX_PWR_CHR) {
        if (ble_uuid_cmp((const ble_uuid_t *)&chr->uuid, BLE_UUID16_DECLARE(ADAPT_BLE_TX_PWR_CHR_UUID)) == 0) {
            g_discovered_tx_pwr_handle = chr->val_handle;
            ESP_LOGI(TAG, "Discovery: TX power chr handle=0x%04X", chr->val_handle);
        }
    } else if (g_disc_state == DISC_STATE_STREAM_CHR) {
        if (ble_uuid_cmp((const ble_uuid_t *)&chr->uuid, BLE_UUID16_DECLARE(ADAPT_BLE_STREAM_CHR_UUID)) == 0) {
            g_discovered_stream_handle = chr->val_handle;
            ESP_LOGI(TAG, "Discovery: stream chr handle=0x%04X", chr->val_handle);
        }
    }

    return 0;
}

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg) {
    (void)arg;

    /* 1* callback: service found */
    if (error->status == 0 && service != NULL) {
        g_disc_svc_start = service->start_handle;
        g_disc_svc_end = service->end_handle;
        return 0;
    }

    /* 2* callback: discovery complete indication via BLE_HS_EDONE */
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "Discovery: service search failed, status=%d", error->status);
        g_disc_state = DISC_STATE_IDLE;
        return 0;
    }

    /* Service discovery complete. Advance the state machine. */
    if (g_disc_state == DISC_STATE_TX_PWR_SVC) {
        if (g_disc_svc_start != 0) {
            g_disc_state = DISC_STATE_TX_PWR_CHR;
            int rc = ble_gattc_disc_all_chrs(conn_handle, g_disc_svc_start, g_disc_svc_end,
                                               disc_chr_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Discovery: failed to start TX-power chr search, rc=%d", rc);
                g_disc_state = DISC_STATE_IDLE;
            }
        } else {
            /* TX-power service not found; try stream service next. */
            g_disc_state = DISC_STATE_STREAM_SVC;
            int rc = ble_gattc_disc_svc_by_uuid(conn_handle,
                                                 BLE_UUID16_DECLARE(ADAPT_BLE_STREAM_SVC_UUID),
                                                 disc_svc_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Discovery: failed to start stream service search, rc=%d", rc);
                g_disc_state = DISC_STATE_IDLE;
            }
        }
    } else if (g_disc_state == DISC_STATE_STREAM_SVC) {
        if (g_disc_svc_start != 0) {
            g_disc_state = DISC_STATE_STREAM_CHR;
            int rc = ble_gattc_disc_all_chrs(conn_handle, g_disc_svc_start, g_disc_svc_end,
                                               disc_chr_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Discovery: failed to start stream chr search, rc=%d", rc);
                g_disc_state = DISC_STATE_IDLE;
            }
        } else {
            g_disc_state = DISC_STATE_DONE;
            ESP_LOGI(TAG,
                     "Discovery complete: tx_pwr_handle=0x%04X, stream_handle=0x%04X",
                     g_discovered_tx_pwr_handle, g_discovered_stream_handle);

            if (g_discovered_stream_handle != 0) {    
                /* Start dummy stream now */
                stream_gatt_start();
            }
        }
    }

    return 0;
}

/*************************************************************************************/
/*                                DUMMY STREAM TASK                                  */
/*************************************************************************************/

/**
 * @brief Background task that emulates a music/audio stream by writing dummy
 *        data to the slave's stream characteristic.
 *
 * Uses Write Without Response so ACL traffic is generated at a predictable rate.
 * Failed writes (host out of buffers, not connected, etc.) are skipped.
 */
static void adapt_ble_stream_task(void *param) {
    (void)param;

    static uint8_t dummy[STREAM_PKT_SIZE];
    uint32_t seq = 0;

    ESP_LOGI(TAG,
             "AdaptBLE stream task started (pkt=%d bytes, period=%d ms)",
             STREAM_PKT_SIZE, STREAM_PERIOD_MS);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));

        uint16_t conn_handle = gap_get_conn_handle();
        if (conn_handle == 0 || conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            continue;
        }

        /* Simple sequence number at the start so the slave can detect gaps */
        memcpy(dummy, &seq, sizeof(seq));
        seq++;

        int rc = ble_gattc_write_no_rsp_flat(conn_handle,
                                              adapt_ble_gatt_get_stream_handle(),
                                              dummy, sizeof(dummy));
        if (rc != 0) {
            ESP_LOGD(TAG, "Stream write skipped: rc=%d", rc);
        }
    }
}

static void stream_gatt_start(void) {
    if (g_stream_task_handle == NULL) {
        xTaskCreate(adapt_ble_stream_task, "AdaptBLE_Stream", 4 * 1024, NULL, 5,
                    &g_stream_task_handle);
    }
}

/*************************************************************************************/
/*                                  PUBLIC FUNCTIONS                                 */
/*************************************************************************************/

void adapt_ble_gatt_start(uint16_t conn_handle) {
    /* Reset discovery state and handles */
    g_discovered_tx_pwr_handle = 0;
    g_discovered_stream_handle = 0;
    g_disc_svc_start = 0;
    g_disc_svc_end = 0;
    g_disc_state = DISC_STATE_TX_PWR_SVC;

    /* Start service discovery */
    int rc = ble_gattc_disc_svc_by_uuid(conn_handle,
                                         BLE_UUID16_DECLARE(ADAPT_BLE_TX_PWR_SVC_UUID),
                                         disc_svc_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Discovery: failed to start, rc=%d", rc);
        g_disc_state = DISC_STATE_IDLE;
    } else {
        ESP_LOGI(TAG, "Discovery: started for AdaptBLE services");
    }
}

void adapt_ble_gatt_stop(void) {
    if (g_stream_task_handle != NULL) {
        vTaskDelete(g_stream_task_handle);
        g_stream_task_handle = NULL;
    }
    g_disc_state = DISC_STATE_IDLE;
}

#endif /* ADAPT_BLE */
