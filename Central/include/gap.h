/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef GAP_SVC_H
#define GAP_SVC_H

/* Includes */
/* NimBLE GAP APIs */
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

/* Defines */
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

#define TARGET_NAME "Peripheral"
#define MAX_DISCOVERED_DEVICES 16

/* GAP state */
typedef enum {
    GAP_STATE_IDLE,
    GAP_STATE_SCANNING,
    GAP_STATE_CONNECTING,
    GAP_STATE_CONNECTED,
} gap_state_t;

/* Public function declarations */
void device_init(void);
int gap_init(void);

gap_state_t gap_get_state(void);
const char *gap_state_str(void);
void gap_get_connected_info(char *addr_str, size_t addr_len, char *name, size_t name_len);

int gap_scan_start(void);
int gap_scan_stop(void);
int gap_connect_addr_str(const char *addr_str);
int gap_disconnect_by_addr_or_name(const char *addr_or_name);

#endif // GAP_SVC_H
