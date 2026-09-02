#ifndef ADAPT_BLE_H
#define ADAPT_BLE_H

/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/

#include "sdkconfig.h"

#ifdef CONFIG_ADAPT_BLE
#define ADAPT_BLE
#endif

#ifdef ADAPT_BLE

#include <stdint.h>
#include <stdbool.h>
#include "esp_nimble_hci.h"

/*************************************************************************************/
/*                                   PUBLIC FUNCTIONS                                */
/*************************************************************************************/


/**
 * @brief captures time instant in which the packet is sent/received from the controller
 * 
 * @param timestamp - either the moment at which the packet is sent to the controller or
 *                    the moment at which packets are forwarded to the host from the controller
 * @param complete - true if controller is sending the packets to the host, false otherwise
 */
void ttx_pending_fifo_push(uint64_t timestamp, bool complete);

/**
 * @brief AdaptBLE latency estimator.
 *
 * Calculates the number of connection intervals a packet spent between
 * host submission and controller buffer release, compensating for the
 * connection event duration.
 *
 * ECI_i = ceil((t_TX - t_CE) / T_CI)
 *
 * @param t_add Timestamp (us) when the packet was submitted to the controller.
 * @param t_free Timestamp (us) when the controller buffer was released.
 * @param t_ce Maximum connection event length in microseconds.
 * @param t_ci Current connection interval in microseconds.
 * @return Estimated ECI_i value.
 */
uint32_t adapt_ble_latency_estimator(uint64_t t_add, uint64_t t_free,
                                     uint32_t t_ce, uint32_t t_ci);

/**
 * @brief Read the latest ACL packet timestamps captured by the HCI layer.
 *
 * The timestamps are taken from the moment the host submits an ACL packet
 * to the controller and the moment the controller reports that the buffer
 * has been freed via HCI_Number_Of_Completed_Packets.
 *
 * @param t_add Output: timestamp (us) when the packet was submitted.
 * @param t_free Output: timestamp (us) when the buffer was freed.
 * @return true if a fresh pair was available and has been copied.
 */
bool adapt_ble_read_acl_timestamps(uint64_t *t_add, uint64_t *t_free);

/**
 * @brief Set the current connection interval used by the latency estimator.
 *
 * Should be updated whenever the BLE connection interval changes.
 *
 * @param t_ci_ms Current connection interval in milliseconds.
 */
void adapt_ble_set_connection_interval(uint32_t t_ci_ms);

/**
 * @brief Start the periodic AdaptBLE adaptation task.
 *
 * The task runs every UPDATE_PERIOD_MS (1 second by default) and calls
 * adapt_ble_run_once().
 */
void adapt_ble_start(void);

/**
 * @brief Send the requested TX power level to the slave via GATT write.
 *
 * The slave must expose a writable characteristic with the handle defined
 * by ADAPT_BLE_SLAVE_TX_PWR_HANDLE. On receiving the value, the slave
 * applies it locally with esp_ble_tx_power_set().
 *
 * @param conn_handle Connection handle of the slave.
 * @param tx_power_dbm Requested TX power in dBm.
 * @return 0 on success; NimBLE error code on failure.
 */
int set_slave_tx_power(uint16_t conn_handle, int8_t tx_power_dbm);

/**
 * @brief Notify the AdaptBLE module that a PHY update has completed.
 *
 * This should be called from the BLE_GAP_EVENT_PHY_UPDATE_COMPLETE handler.
 * It is used to commit the pending LUT position only after the slave has
 * acknowledged the PHY change.
 *
 * @param status 0 on success, BLE error code on failure.
 */
void adapt_ble_on_phy_update_complete(uint8_t status);

#endif /* ADAPT_BLE */

#endif /* ADAPT_BLE_H */
