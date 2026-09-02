#ifndef ADAPT_BLE_GATT_H
#define ADAPT_BLE_GATT_H

/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/
#include "common.h"

#ifdef ADAPT_BLE

#include <stdint.h>

/*************************************************************************************/
/*                                   PUBLIC FUNCTIONS                                */
/*************************************************************************************/

/**
 * @brief Start AdaptBLE GATT discovery and the dummy stream task.
 *
 * Discovers the AdaptBLE TX-power and dummy-stream characteristics on the
 * slave. Until discovery completes, writes fall back to the handles configured
 * in menuconfig.
 *
 * @param conn_handle Active connection handle to the slave.
 */
void adapt_ble_gatt_start(uint16_t conn_handle);

/**
 * @brief Stop the dummy stream task and reset discovery state.
 */
void adapt_ble_gatt_stop(void);

/**
 * @brief Return the discovered TX-power characteristic handle.
 *
 * @return Discovered handle, or the configured fallback if discovery has not
 *         completed.
 */
uint16_t adapt_ble_gatt_get_tx_pwr_handle(void);

/**
 * @brief Return the discovered dummy-stream characteristic handle.
 *
 * @return Discovered handle, or the configured fallback if discovery has not
 *         completed.
 */
uint16_t adapt_ble_gatt_get_stream_handle(void);

#endif /* ADAPT_BLE */

#endif /* ADAPT_BLE_GATT_H */
