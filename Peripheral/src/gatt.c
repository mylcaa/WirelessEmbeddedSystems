/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/
#include "gatt.h"
#include "common.h"

#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "esp_bt.h"

/*************************************************************************************/
/*                          PRIVATE DEFINITIONS AND TYPES                            */
/*************************************************************************************/

/* 16-bit custom UUIDs for the AdaptBLE TX-power service */
#define ADAPT_BLE_TX_PWR_SVC_UUID 0xAD80
#define ADAPT_BLE_TX_PWR_CHR_UUID 0xAD81

/* Variable that receives the assigned characteristic value handle */
static uint16_t tx_pwr_val_handle;

/*************************************************************************************/
/*                                 PRIVATE FUNCTIONS                                 */
/*************************************************************************************/

static esp_power_level_t dbm_to_esp_power_level(int8_t dbm) {
    int level = (dbm + 24) / 3;
    if (level < 0) {
        level = 0;
    } else if (level > 15) {
        level = 15;
    }
    return (esp_power_level_t)level;
}

static int tx_pwr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    int8_t pwr = 0;
    uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
    if (om_len != sizeof(pwr)) {
        ESP_LOGE(TAG,
                 "TX power write: invalid length %d, expected %d",
                 om_len, (int)sizeof(pwr));
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    os_mbuf_copydata(ctxt->om, 0, sizeof(pwr), &pwr);

    ESP_LOGI(TAG, "Received TX power request: %d dBm", pwr);

    esp_err_t err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,
                                         dbm_to_esp_power_level(pwr));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply TX power %d dBm: %d", pwr, err);
        return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGI(TAG, "Applied TX power %d dBm", pwr);
    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(ADAPT_BLE_TX_PWR_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(ADAPT_BLE_TX_PWR_CHR_UUID),
                .access_cb = tx_pwr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &tx_pwr_val_handle,
            },
            { 0 }
        }
    },
    { 0 }
};


/*************************************************************************************/
/*                                  PUBLIC FUNCTIONS                                 */
/*************************************************************************************/

uint16_t gatt_svr_get_tx_pwr_handle(void) {
    return tx_pwr_val_handle;
}

void gatt_svr_init(void) {
    int rc = 0;

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT service config, rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services, rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG,
             "AdaptBLE TX-power GATT service registered (svc=0x%04X, chr=0x%04X)",
             ADAPT_BLE_TX_PWR_SVC_UUID, ADAPT_BLE_TX_PWR_CHR_UUID);
}
