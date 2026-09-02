/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/
#include "adapt_ble.h"
#include "common.h"
#include "gap.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "esp_bt.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#ifdef ADAPT_BLE

/*************************************************************************************/
/*                               DEFINITIONS AND TYPES                               */
/*************************************************************************************/

#define LATENCY_THRESH_MS          100
#define UPDATE_PERIOD_MS           1000
#define NUM_MEASUREMENTS           50
#define LINK_QUALITY_TH_LOW        0.05f
#define LINK_QUALITY_TH_HIGH       0.2f
#define MAX_CONN_EVENT_DURATION_MS 10

/* Convert configuration constants to microseconds */
#define LATENCY_THRESH_US          (LATENCY_THRESH_MS * 1000U)
#define MAX_CONN_EVENT_DURATION_US (MAX_CONN_EVENT_DURATION_MS * 1000U)
#define UPDATE_PERIOD_US           (UPDATE_PERIOD_MS * 1000U)

#define DATA_RATE_NUM        4
#define TRANSMISSION_PWR_NUM 10

#define TTX_FIFO_SIZE 50

/* Handle for the slave TX-power GATT characteristic (from menuconfig) */
#ifdef CONFIG_ADAPT_BLE_SLAVE_TX_PWR_HANDLE
#define ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE CONFIG_ADAPT_BLE_SLAVE_TX_PWR_HANDLE
#else
#define ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE 0x002A
#endif

typedef enum {
    DATA_RATE_2M = 0, /* BLE_HCI_LE_PHY_2M */
    DATA_RATE_1M,     /* BLE_HCI_LE_PHY_1M */
    DATA_RATE_500k,   /* BLE_HCI_LE_PHY_CODED_S2 */
    DATA_RATE_125k    /* BLE_HCI_LE_PHY_CODED_S8 */
} data_rate_t;

typedef struct {
    data_rate_t data_rate;
    int8_t tx_pwr;
} lut_t;

typedef struct {
    int8_t y;
    int8_t x;
} lut_index_t;

/*
 * Energy LUT: rows are data rates (fastest at top), columns are TX power
 * levels (lowest at left). The upper-left corner is lowest energy,
 * the lower-right corner is highest energy (best link quality).
 */
static const lut_t energy_consumption_lut[DATA_RATE_NUM][TRANSMISSION_PWR_NUM] = {
    {{DATA_RATE_2M, -21}, {DATA_RATE_2M, -18}, {DATA_RATE_2M, -15}, {DATA_RATE_2M, -12}, {DATA_RATE_2M, -9}, {DATA_RATE_2M, -6}, {DATA_RATE_2M, -3}, {DATA_RATE_2M, 0}, {DATA_RATE_2M, 3}, {DATA_RATE_2M, 6}},
    {{DATA_RATE_1M, -21}, {DATA_RATE_1M, -18}, {DATA_RATE_1M, -15}, {DATA_RATE_1M, -12}, {DATA_RATE_1M, -9}, {DATA_RATE_1M, -6}, {DATA_RATE_1M, -3}, {DATA_RATE_1M, 0}, {DATA_RATE_1M, 3}, {DATA_RATE_1M, 6}},
    {{DATA_RATE_500k, -21}, {DATA_RATE_500k, -18}, {DATA_RATE_500k, -15}, {DATA_RATE_500k, -12}, {DATA_RATE_500k, -9}, {DATA_RATE_500k, -6}, {DATA_RATE_500k, -3}, {DATA_RATE_500k, 0}, {DATA_RATE_500k, 3}, {DATA_RATE_500k, 6}},
    {{DATA_RATE_125k, -21}, {DATA_RATE_125k, -18}, {DATA_RATE_125k, -15}, {DATA_RATE_125k, -12}, {DATA_RATE_125k, -9}, {DATA_RATE_125k, -6}, {DATA_RATE_125k, -3}, {DATA_RATE_125k, 0}, {DATA_RATE_125k, 3}, {DATA_RATE_125k, 6}}
};

typedef struct {
    uint64_t t_add;
    uint64_t t_free;
    bool complete;
} timestamp_t;

static timestamp_t ttx_pending_fifo[TTX_FIFO_SIZE] = {{0, 0, false}};
static uint8_t add_fit = 0;   /* next slot to write a t_add */
static uint8_t free_fit = 0;  /* oldest pending t_add to mark with t_free */
static uint8_t tx_fit = 0;    /* next completed entry to read */
static uint8_t pending_count = 0;  /* t_add submitted but not completed */
static uint8_t complete_count = 0; /* completed entries ready to read */

static SemaphoreHandle_t ttx_fifo_mutex = NULL;

/* ECI measurement history (circular buffer) */
static uint8_t ECI[NUM_MEASUREMENTS] = {0};
static int8_t g_curr_ECI = 0;
static uint32_t g_round_index = 0;

/* Current operating point in the energy LUT */
static data_rate_t g_data_rate = DATA_RATE_1M;
static int8_t g_tx_pwr = 0;
static lut_index_t g_curr_pos = {1, 7};

/* Pending LUT update: do not commit g_curr_pos until slave confirms */
static lut_index_t g_pending_pos = {1, 7};
static bool g_phy_pending = false;
static bool g_txpwr_pending = false;

/* Connection interval state */
static uint32_t g_conn_interval_us = 400 * 1000U; /* default 400 ms */

/*************************************************************************************/
/*                                  HELPER FUNCTIONS                                 */
/*************************************************************************************/

static inline uint32_t ceil_div(uint32_t a, uint32_t b) {
    if (b == 0) {
        return 0;
    }
    return (a + b - 1) / b;
}

static uint8_t data_rate_to_phy_mask(data_rate_t rate) {
    switch (rate) {
    case DATA_RATE_2M:
        return BLE_HCI_LE_PHY_2M_PREF_MASK;
    case DATA_RATE_1M:
        return BLE_HCI_LE_PHY_1M_PREF_MASK;
    case DATA_RATE_500k:
    case DATA_RATE_125k:
        return BLE_HCI_LE_PHY_CODED_PREF_MASK;
    default:
        return BLE_HCI_LE_PHY_1M_PREF_MASK;
    }
}

static uint16_t data_rate_to_phy_options(data_rate_t rate) {
    switch (rate) {
    case DATA_RATE_500k:
        return BLE_HCI_LE_PHY_CODED_S2;
    case DATA_RATE_125k:
        return BLE_HCI_LE_PHY_CODED_S8;
    default:
        return BLE_HCI_LE_PHY_CODED_ANY;
    }
}

static uint8_t find_max_eci(void) {
    uint8_t eci_max = 0;

    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        if (ECI[i] > eci_max) {
            eci_max = ECI[i];
        }
    }

    return eci_max;
}

static float retransmission_ratio_calc(void) {
    uint32_t num_retransmissions = 0;

    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        if (ECI[i] > 1) {
            num_retransmissions++;
        }
    }

    return (float)num_retransmissions / NUM_MEASUREMENTS;
}

/*************************************************************************************/
/*               DATA RATE AND TRANSMISSION POWER MANAGER FUNCTIONS                */
/*************************************************************************************/

static void update_lut_settings(void) {
    g_data_rate = energy_consumption_lut[g_curr_pos.y][g_curr_pos.x].data_rate;
    g_tx_pwr = energy_consumption_lut[g_curr_pos.y][g_curr_pos.x].tx_pwr;
}

static void abort_pending_lut_update(const char *reason);
static void try_commit_lut_position(void);

/**
 * @brief mapping dBm to the correct enumeration value defined in esp_bt.h
 */
static esp_power_level_t dbm_to_esp_power_level(int8_t dbm) {
    int level = (dbm + 24) / 3;
    if (level < 0) {
        level = 0;
    } else if (level > 15) {
        level = 15;
    }
    return (esp_power_level_t)level;
}

static int set_slave_tx_power_cb(uint16_t conn_handle,
                                    const struct ble_gatt_error *error,
                                    struct ble_gatt_attr *attr, void *arg) {
    (void)conn_handle;
    (void)attr;

    int8_t pwr = (int8_t)(intptr_t)arg;

    if (error == NULL || error->status != 0) {
        ESP_LOGE(TAG,
                 "Slave TX power write failed: pwr=%d dBm, status=%d",
                 pwr, error ? error->status : -1);
        abort_pending_lut_update("TX power write rejected");
        return -1;
    }

    ESP_LOGI(TAG,
             "Slave acknowledged TX power %d dBm; applying locally", pwr);

    /* Apply the same TX power on the master side */
    esp_err_t err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,
                                         dbm_to_esp_power_level(pwr));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply local TX power %d dBm: %d", pwr, err);
        abort_pending_lut_update("local TX power apply failed");
        return -1;
    }

    g_txpwr_pending = false;
    try_commit_lut_position();

    return 0;
}

int set_slave_tx_power(uint16_t conn_handle, int8_t tx_power_dbm) {
    int rc = ble_gattc_write_flat(conn_handle,
                                   ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE,
                                   &tx_power_dbm, sizeof(tx_power_dbm),
                                   set_slave_tx_power_cb,
                                   (void *)(intptr_t)tx_power_dbm);
    if (rc != 0) {
        ESP_LOGE(TAG,
                 "set_slave_tx_power failed: handle=0x%04X, pwr=%d dBm, rc=%d",
                 ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE, tx_power_dbm, rc);
    } else {
        ESP_LOGI(TAG, "Sent TX power %d dBm to slave (handle=0x%04X)",
                 tx_power_dbm, ADAPT_BLE_SLAVE_TX_PWR_ATTR_HANDLE);
    }
    return rc;
}

static int set_slave_phy(uint16_t conn_handle, data_rate_t rate) {
    uint8_t phy_mask = data_rate_to_phy_mask(rate);
    uint16_t phy_opts = data_rate_to_phy_options(rate);

    int rc = ble_gap_set_prefered_le_phy(conn_handle, phy_mask, phy_mask,
                                         phy_opts);
    if (rc != 0) {
        ESP_LOGE(TAG, "set_slave_phy failed: rate=%d, rc=%d", (int)rate, rc);
    } else {
        ESP_LOGI(TAG, "Sent PHY preference 0x%02X (opts=%u) to slave",
                 phy_mask, (unsigned)phy_opts);
    }
    return rc;
}

static void abort_pending_lut_update(const char *reason) {
    ESP_LOGW(TAG, "Aborting pending LUT update: %s", reason ? reason : "unknown");
    g_pending_pos = g_curr_pos;
    g_phy_pending = false;
    g_txpwr_pending = false;
}

static void try_commit_lut_position(void) {
    if (g_phy_pending || g_txpwr_pending) {
        return;
    }
    if (g_pending_pos.x == g_curr_pos.x && g_pending_pos.y == g_curr_pos.y) {
        return;
    }

    g_curr_pos = g_pending_pos;
    update_lut_settings();

    ESP_LOGI(TAG,
             "Committed LUT position: L_DR=%d, L_TP=%d, rate=%d, pwr=%d",
             g_curr_pos.y, g_curr_pos.x, (int)g_data_rate, g_tx_pwr);
}

static void request_lut_update(lut_index_t new_pos) {
    if (g_phy_pending || g_txpwr_pending) {
        ESP_LOGW(TAG, "LUT update already pending; skipping new request");
        return;
    }
    if (new_pos.x == g_curr_pos.x && new_pos.y == g_curr_pos.y) {
        return;
    }

    uint16_t conn_handle = gap_get_conn_handle();
    if (conn_handle == 0 || conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "No connection; skipping LUT update");
        return;
    }

    g_pending_pos = new_pos;
    g_phy_pending = true;
    g_txpwr_pending = true;

    data_rate_t pending_rate =
        energy_consumption_lut[g_pending_pos.y][g_pending_pos.x].data_rate;
    int8_t pending_pwr =
        energy_consumption_lut[g_pending_pos.y][g_pending_pos.x].tx_pwr;

    ESP_LOGI(TAG,
             "Requesting LUT move to L_DR=%d, L_TP=%d (rate=%d, pwr=%d)",
             g_pending_pos.y, g_pending_pos.x,
             (int)pending_rate, pending_pwr);

    int rc = set_slave_phy(conn_handle, pending_rate);
    if (rc != 0) {
        abort_pending_lut_update("PHY request rejected");
        return;
    }

    rc = set_slave_tx_power(conn_handle, pending_pwr);
    if (rc != 0) {
        abort_pending_lut_update("TX power request rejected");
        return;
    }
}

static void increase_link_quality(void) {
    lut_index_t new_pos = g_curr_pos;

    if (new_pos.x < (TRANSMISSION_PWR_NUM - 1)) {
        new_pos.x++;
    } else if (new_pos.y < (DATA_RATE_NUM - 1)) {
        new_pos.y++;
    } else {
        /* Already at the highest energy point */
        return;
    }

    request_lut_update(new_pos);
}

static void decrease_energy_consumption(void) {
    lut_index_t new_pos = g_curr_pos;

    if (new_pos.y > 0) {
        new_pos.y--;
    } else if (new_pos.x > 0) {
        new_pos.x--;
    } else {
        /* Already at the lowest energy point */
        return;
    }

    request_lut_update(new_pos);
}

static void data_rate_and_tx_pwr_manager(void) {
    float rt_ratio = retransmission_ratio_calc();

    if (rt_ratio > LINK_QUALITY_TH_HIGH) {
        increase_link_quality();
    } else if (rt_ratio < LINK_QUALITY_TH_LOW) {
        decrease_energy_consumption();
    }
}

void adapt_ble_on_phy_update_complete(uint8_t status) {
    if (status != 0) {
        ESP_LOGE(TAG, "PHY update failed: status=%d", status);
        abort_pending_lut_update("PHY update failed");
        return;
    }

    ESP_LOGI(TAG, "PHY update acknowledged by slave");
    g_phy_pending = false;
    try_commit_lut_position();
}

/*************************************************************************************/
/*                             LATENCY ESTIMATOR FUNCTIONS                           */
/*************************************************************************************/

uint32_t adapt_ble_latency_estimator(uint64_t t_add, uint64_t t_free,
                                     uint32_t t_ce, uint32_t t_ci) {
    if (t_free <= t_add || t_ci == 0) {
        return 0;
    }

    uint32_t t_tx = (uint32_t)(t_free - t_add);

    if (t_tx <= t_ce) {
        return 0;
    }

    return ceil_div(t_tx - t_ce, t_ci);
}

/*************************************************************************************/
/*                        CONNECTION INTERVAL FUNCTIONS                              */
/*************************************************************************************/

static void apply_connection_interval(uint16_t conn_handle,
                                       uint32_t recommended_us) {

    /* ble_gap_update_params expects interval in 1.25 ms units */
    uint16_t itvl_units = (uint16_t)((recommended_us + 625U) / 1250U);
    if (itvl_units < 6) {
        itvl_units = 6; /* BLE minimum: 7.5 ms */
    } else if (itvl_units > 3200) {
        itvl_units = 3200; /* BLE maximum: 4 s */
    }

    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGE(TAG, "apply_connection_interval: conn_find failed, rc=%d", rc);
        return;
    }

    struct ble_gap_upd_params params = {
        .itvl_min = itvl_units,
        .itvl_max = itvl_units,
        .latency = desc.conn_latency,
        .supervision_timeout = desc.supervision_timeout,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    rc = ble_gap_update_params(conn_handle, &params);
    if (rc != 0) {
        ESP_LOGE(TAG,
                 "ble_gap_update_params failed: target=%u us (%u units), rc=%d",
                 (unsigned)recommended_us, itvl_units, rc);
    } else {
        ESP_LOGI(TAG,
                 "Requested connection interval update: %u us (%u units)",
                 (unsigned)recommended_us, itvl_units);
    }
}

/*************************************************************************************/
/*                                PUBLIC FUNCTIONS                                   */
/*************************************************************************************/

void ttx_pending_fifo_push(uint64_t timestamp, bool complete) {
    if (ttx_fifo_mutex == NULL) {
        return;
    }

    xSemaphoreTake(ttx_fifo_mutex, portMAX_DELAY);

    if (!complete) {
        /* t_add: make room if the FIFO is full */
        if ((pending_count + complete_count) >= TTX_FIFO_SIZE) {
            ESP_LOGW(TAG, "TTX FIFO full; dropping oldest sample");
            if (complete_count > 0) {
                tx_fit = (tx_fit + 1) % TTX_FIFO_SIZE;
                complete_count--;
            } else if (pending_count > 0) {
                free_fit = (free_fit + 1) % TTX_FIFO_SIZE;
                pending_count--;
            }
        }

        ttx_pending_fifo[add_fit].t_add = timestamp;
        ttx_pending_fifo[add_fit].t_free = 0;
        ttx_pending_fifo[add_fit].complete = false;
        add_fit = (add_fit + 1) % TTX_FIFO_SIZE;
        pending_count++;
    } else {
        /* t_free: pair with the oldest pending t_add */
        if (pending_count == 0) {
            ESP_LOGW(TAG, "TTX FIFO: t_free without pending t_add, ignoring");
        } else {
            ttx_pending_fifo[free_fit].t_free = timestamp;
            ttx_pending_fifo[free_fit].complete = true;
            free_fit = (free_fit + 1) % TTX_FIFO_SIZE;
            pending_count--;
            complete_count++;
        }
    }

    xSemaphoreGive(ttx_fifo_mutex);
}

bool adapt_ble_read_acl_timestamps(uint64_t *t_add, uint64_t *t_free) {
    if (ttx_fifo_mutex == NULL || t_add == NULL || t_free == NULL) {
        return false;
    }

    xSemaphoreTake(ttx_fifo_mutex, portMAX_DELAY);

    if (complete_count == 0) {
        xSemaphoreGive(ttx_fifo_mutex);
        ESP_LOGD(TAG, "No new NCP measurement");
        return false;
    }

    *t_add = ttx_pending_fifo[tx_fit].t_add;
    *t_free = ttx_pending_fifo[tx_fit].t_free;
    ttx_pending_fifo[tx_fit] = (timestamp_t){0, 0, false};
    tx_fit = (tx_fit + 1) % TTX_FIFO_SIZE;
    complete_count--;

    xSemaphoreGive(ttx_fifo_mutex);
    return true;
}

void adapt_ble_set_connection_interval(uint32_t t_ci_ms) {
    g_conn_interval_us = t_ci_ms * 1000U;
}

/*************************************************************************************/
/*                                ADAPT BLE ALGORITHM                                */
/*************************************************************************************/

static void adapt_ble_run_once(void) {
    /* Step 1: estimate current ECI from a fresh ACL timestamp pair */
    uint64_t t_add;
    uint64_t t_free;

    if(!adapt_ble_read_acl_timestamps(&t_add, &t_free)) {
        return;
    }

    /* Step 2: compute ECI_max over the last M rounds and calculate new T_CI */
    uint32_t recommended_ci_us = 0;
    uint16_t conn_handle = gap_get_conn_handle();
    uint8_t eci_max = find_max_eci();

    if (eci_max != 0) {
        recommended_ci_us = (LATENCY_THRESH_US - MAX_CONN_EVENT_DURATION_US) / eci_max;
    }

    ESP_LOGI(TAG,
             "AdaptBLE round %lu: ECI_max=%u, recommended T_CI=%lu us",
             (unsigned long)g_round_index, eci_max,
             (unsigned long)recommended_ci_us);

    /* Step 3: every M rounds, run the data-rate / TX-power manager */
    if ((g_round_index % NUM_MEASUREMENTS) == 0) {
        data_rate_and_tx_pwr_manager();
        ESP_LOGI(TAG,
                 "AdaptBLE LUT update: L_DR=%d, L_TP=%d, data_rate=%d, tx_pwr=%d",
                 g_curr_pos.y, g_curr_pos.x, (int)g_data_rate, g_tx_pwr);
    }

    /* Step 4: push recommended connection interval to the slave */
    if (conn_handle != 0 && conn_handle != BLE_HS_CONN_HANDLE_NONE
        && recommended_ci_us != 0 && recommended_ci_us != g_conn_interval_us) 
    {
        apply_connection_interval(conn_handle, recommended_ci_us);
    }

    /* Step 5: store the newly measured ECI for the next round */
    uint32_t eci = adapt_ble_latency_estimator(t_add, t_free,
                                                MAX_CONN_EVENT_DURATION_US,
                                                g_conn_interval_us);
    ECI[g_curr_ECI] = (uint8_t)eci;
    g_curr_ECI = (g_curr_ECI + 1) % NUM_MEASUREMENTS;

    g_round_index++;
}

static void adapt_ble_task(void *param) {
    (void)param;

    ESP_LOGI(TAG, "AdaptBLE periodic task started (period=%d ms)", UPDATE_PERIOD_MS);

    for (;;) {
        adapt_ble_run_once();
        vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
    }
}

void adapt_ble_start(void) {
    ttx_fifo_mutex = xSemaphoreCreateMutex();
    if (ttx_fifo_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create TTX FIFO mutex");
        return;
    }

    xTaskCreate(adapt_ble_task, "AdaptBLE", 4 * 1024, NULL, 5, NULL);
}

#endif /* ADAPT_BLE */
