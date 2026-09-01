/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/

#include "esp_nimble_hci.h"

#ifdef ADAPT_BLE

/*************************************************************************************/
/*                               DEFINITIONS AND TYPES                               */
/*************************************************************************************/

#define LATENCY_THRESH_MS          100 
#define UPDATE_PERIOD_MS           1000
#define NUM_MEASUREMENTS           50
#define LINK_QUALITY_TH_LOW        0.05
#define LINK_QUALITY_TH_HIGH       0.2
#define MAX_CONN_EVENT_DURATION_MS 10

#define DATA_RATE_NUM        4
#define TRANSMISSION_PWR_NUM 10

static uint8_t ECI[NUM_MEASUREMENTS] = {0};
static int8_t g_curr_ECI = 0;

typedef enum {
    DATA_RATE_2M = 0,
    DATA_RATE_1M,
    DATA_RATE_500k,
    DATA_RATE_125k
} data_rate_t;

typedef struct {
    data_rate_t data_rate;
    int8_t tx_pwr;
} lut_t

typedef struct {
    int y;
    int x;
} lut_index_t;

// The lower right edge of the table represents high energy consumption while the upper left edge represents low energy consumption 
lut_t energy_consumption_lut[DATA_RATE_NUM][TRANSMISSION_PWR_NUM] = {{{DATA_RATE_2M, -21}, {DATA_RATE_2M, -18}, {DATA_RATE_2M, -15}, {DATA_RATE_2M, -12}, {DATA_RATE_2M, -9}, {DATA_RATE_2M, -6}, {DATA_RATE_2M, -3}, {DATA_RATE_2M, 0}, {DATA_RATE_2M, 2}, {DATA_RATE_2M, 5}},
                                                                     {{DATA_RATE_1M, -21}, {DATA_RATE_1M, -18}, {DATA_RATE_1M, -15}, {DATA_RATE_1M, -12}, {DATA_RATE_1M, -9}, {DATA_RATE_1M, -6}, {DATA_RATE_1M, -3}, {DATA_RATE_1M, 0}, {DATA_RATE_1M, 2}, {DATA_RATE_1M, 5}},
                                                                     {{DATA_RATE_500k, -21}, {DATA_RATE_500k, -18}, {DATA_RATE_500k, -15}, {DATA_RATE_500k, -12}, {DATA_RATE_500k, -9}, {DATA_RATE_500k, -6}, {DATA_RATE_500k, -3}, {DATA_RATE_500k, 0}, {DATA_RATE_500k, 2}, {DATA_RATE_500k, 5}},
                                                                     {{DATA_RATE_125k, -21}, {DATA_RATE_125k, -18}, {DATA_RATE_125k, -15}, {DATA_RATE_125k, -12}, {DATA_RATE_125k, -9}, {DATA_RATE_125k, -6}, {DATA_RATE_125k, -3}, {DATA_RATE_125k, 0}, {DATA_RATE_125k, 2}, {DATA_RATE_125k, 5}}}


//Default connection parameters
static data_rate_t g_data_rate = DATA_RATE_1M;
static int8_t g_tx_pwr = 0;
static lut_index_t g_curr_pos = {1, 7};

static float g_conn_interval = 400; //milliseconds

/*************************************************************************************/
/*                                  HELPER FUNCTIONS                                 */
/*************************************************************************************/

static inline uint32_t ceil_div(uint_addt32_t a, uint32_t b) {
    if (b == 0) {
        return 0;
    }
    return (a + b - 1) / b;
}

/*************************************************************************************/
/*                             LATENCY ESTIMATOR FUNCTIONS                           */
/*************************************************************************************/

static void adapt_ble_latency_estimator(uint64_t t_add, uint64_t t_free) {
    uint64_t t_tx = t_add - t_free;

    if (t_tx <= MAX_CONN_EVENT_DURATION_MS || t_ci == 0) {
        //log issue
        return;
    }

    ECI[g_curr_ECI] = ceil_div((t_tx - MAX_CONN_EVENT_DURATION_MS) / g_conn_interval);
    
    if (g_curr_ECI < NUM_MEASUREMENTS) {
        g_curr_ECI++;
    } else {
        g_curr_ECI = 0;
    }
}

uint32_t adapt_ble_connection_interval_calculator(uint32_t eci_max) {
    if (eci_max == 0) {
        return 0;
    }

    return (LATENCY_THRESH_MS - MAX_CONN_EVENT_DURATION_MS) / eci_max;
}


/*************************************************************************************/
/*               DATA RATE AND TRANSMISSION POWER MANAGER FUNCTIONS                  */
/*************************************************************************************/

static uint8_t find_max() {

    uint8_t ECI_max = ECI[0];

    for (int i = 1; i < NUM_MEASUREMENTS; ++ i) {
        if(ECI[i] > ECI_max) {
            ECI_max = ECI[i];
        }
    }

    return ECI_max;
}

static float freq_retransmissions_calc() {
    uint8_t num_retransmission = 0;
    for (int i = 0; i < NUM_MEASUREMENTS; ++ i) {
        num_retransmission += ((ECI[i] > 1) ? 1 : 0);
    }

    return float(num_retransmissions)/NUM_MEASUREMENTS;
}

static void increase_link_quality() {
    if (g_curr_pos.x < (TRANSMISSION_PWR_NUM-1)) {
        g_curr_pos.x++;
    } else if (g_curr_pos.y < (DATA_RATE_NUM-1)) {
        g_curr_pos.y++;
    } else {
        //we are already at transmission power and data rate which cause the highest energy consumption
        return;
    }

    g_data_rate = energy_consumption_lut[g_curr_pos.y][g_curr_pos.x].data_rate;
    g_tx_pwr = energy_consumption_lut[g_curr_pos.y][g_curr_pos.x].tx_pwr;
}

static void decrease_energy_consumption() {
    if (g_curr_pos.y > 0) {
        g_curr_pos.y--;
    } else if (g_curr_pos.x > 0) {
        g_curr_pos.x--;
    } else {
        //we are already at transmission power and data rate which cause the lowest energy consumption
        return;
    }

    g_data_rate = energy_consumption_lut[g_curr_pos.y][g_curr_pos.x].data_rate;
    g_tx_pwr = energy_consumption_lut[g_curr_pos.y][g_curr_pos.x].tx_pwr;
}

static void data_rate_and_tx_pwr_manager() {
    float rt_ratio = freq_retransmission_calc(); 
    if(rt_ratio > LINK_QUALITY_TH_HIGH) {
        increase_link_quality();
    } else if (rt_ration < LINK_QUALITY_TH_LOW) {
        decrease_energy_consumption();
    }
}

/*************************************************************************************/
/*                                PUBLIC FUNCTIONS                                   */
/*************************************************************************************/


bool adapt_ble_read_acl_timestamps(uint64_t *t_add, uint64_t *t_free) {
    if (t_add == NULL || t_free == NULL) {
        return false;
    }

    if (!adapt_ble_acl_timestamps_ready()) {
        return false;
    }

    *t_add = adapt_ble_get_acl_t_add();
    *t_free = adapt_ble_get_acl_t_free();
    return true;
}


#endif /* ADAPT_BLE */