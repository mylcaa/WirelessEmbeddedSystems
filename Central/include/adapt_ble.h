/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef ADAPT_BLE_H
#define ADAPT_BLE_H

/* Includes */
#include "sdkconfig.h"

#ifdef CONFIG_ADAPT_BLE
#define ADAPT_BLE
#endif

#ifdef ADAPT_BLE

#include <stdint.h>
#include <stdbool.h>
#include "esp_nimble_hci.h"

/**
 * @brief latency estimator
 * 
 * @param t_add the moment in which the packet is sent from the host to the controller
 * @param t_free the moment in which the buffer allocated in the controller for the packet is freed
 * @param t_ce length of connection event
 * @param t_ci length of connection interval
 * 
 * @retval ECI the number of transmissions required to send the packet 
 */
uint32_t adapt_ble_latency_estimator(uint64_t t_add, uint64_t t_free,
                                     uint32_t t_ce, uint32_t t_ci);

/**
 * @brief AdaptBLE connection interval calculator.
 *
 * Computes a new connection interval T_CI based on the worst-case ECI_max
 * observed over the recent M rounds and a latency threshold t_th.
 *
 * T_CI = floor((t_th - t_CE) / ECI_max)
 *
 * @param t_ce Maximum connection event length in microseconds.
 * @param eci_max Maximum ECI value over the recent M rounds.
 * @return New connection interval T_CI in microseconds.
 */
uint32_t adapt_ble_connection_interval_calculator(uint32_t t_ce,
                                                  uint32_t eci_max);

/**
 * @brief get timestamps for calculating ECI of a packet
 *
 * @param t_add Output: timestamp (us) when the packet was submitted.
 * @param t_free Output: timestamp (us) when the buffer was freed.
 * @return true if a new pair was available and has been copied.
 */
bool adapt_ble_read_acl_timestamps(uint64_t *t_add, uint64_t *t_free);

#endif /* ADAPT_BLE */

#endif /* ADAPT_BLE_H */
