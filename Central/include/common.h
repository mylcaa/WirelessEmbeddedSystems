
#ifndef COMMON_H
#define COMMON_H

/*************************************************************************************/
/*                                     INCLUDES                                      */
/*************************************************************************************/

/* STD APIs */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

/* ESP APIs */
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* FreeRTOS APIs */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* NimBLE stack APIs */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

/*************************************************************************************/
/*                                  DEFINITIONS                                      */
/*************************************************************************************/

#define TAG "Central"
#define DEVICE_NAME "Central"

#endif // COMMON_H
