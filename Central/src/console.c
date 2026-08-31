/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "console.h"
#include "common.h"
#include "gap.h"

#include "esp_console.h"
#include "argtable3/argtable3.h"

/*************************************************************************************/
/*                          PRIVATE DEFINITIONS AND TYPES                            */
/*************************************************************************************/

/* Private function declarations */
static int cmd_scan(int argc, char **argv);
static int cmd_stop_scan(int argc, char **argv);
static int cmd_connect(int argc, char **argv);
static int cmd_disconnect(int argc, char **argv);
static int cmd_get_state(int argc, char **argv);

static struct {
    struct arg_str *addr;
    struct arg_end *end;
} connect_args;

static struct {
    struct arg_str *addr_or_name;
    struct arg_end *end;
} disconnect_args;

/*************************************************************************************/
/*                               CALLBACK FUNCTIONS                                  */
/*************************************************************************************/

/* Private functions */
static int cmd_scan(int argc, char **argv) {
    int rc = gap_scan_start();
    if (rc == 0) {
        ESP_LOGI(TAG, "Scanning started");
    } else {
        ESP_LOGE(TAG, "Failed to start scanning");
    }
    return rc;
}

static int cmd_stop_scan(int argc, char **argv) {
    int rc = gap_scan_stop();
    if (rc == 0) {
        ESP_LOGI(TAG, "Scanning stopped");
    } else {
        ESP_LOGE(TAG, "Failed to stop scanning");
    }
    return rc;
}

static int cmd_connect(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&connect_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, connect_args.end, argv[0]);
        return 1;
    }

    const char *addr = connect_args.addr->sval[0];
    int rc = gap_connect_addr_str(addr);
    if (rc == 0) {
        ESP_LOGI(TAG, "Connecting to %s", addr);
    } else {
        ESP_LOGE(TAG, "Failed to connect to %s", addr);
    }
    return rc;
}

static int cmd_disconnect(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&disconnect_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, disconnect_args.end, argv[0]);
        return 1;
    }

    const char *addr_or_name = disconnect_args.addr_or_name->sval[0];
    int rc = gap_disconnect_by_addr_or_name(addr_or_name);
    if (rc == 0) {
        ESP_LOGI(TAG, "Disconnected from %s", addr_or_name);
    } else {
        ESP_LOGE(TAG, "Failed to disconnect from %s", addr_or_name);
    }
    return rc;
}

static int cmd_get_state(int argc, char **argv) {
    char addr_str[18] = {0};
    char name[32] = {0};
    gap_get_connected_info(addr_str, sizeof(addr_str), name, sizeof(name));

    if (gap_get_state() == GAP_STATE_CONNECTED) {
        printf("State: connected\n");
        printf("Connected to: %s\n", addr_str);
        if (name[0] != '\0') {
            printf("Device name: %s\n", name);
        } else {
            printf("Device name: (unknown)\n");
        }
    } else {
        printf("State: %s\n", gap_state_str());
    }

    return 0;
}

/*************************************************************************************/
/*                                  INIT FUNCTIONS                                   */
/*************************************************************************************/

/* Public functions */
void console_init(void) {
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "Central> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_uart_config_t uart_config =
        ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_repl_t *repl = NULL;
    ESP_ERROR_CHECK(
        esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    /* Register help command */
    esp_console_register_help_command();

    /* scan command */
    esp_console_cmd_t cmd_scan_cfg = {
        .command = "scan",
        .help = "Start scanning for BLE peripherals",
        .func = cmd_scan,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_scan_cfg));

    /* stop_scan command */
    esp_console_cmd_t cmd_stop_scan_cfg = {
        .command = "stop_scan",
        .help = "Stop scanning for BLE peripherals",
        .func = cmd_stop_scan,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_stop_scan_cfg));

    /* connect command */
    connect_args.addr = arg_str1(NULL, NULL, "<address>",
                                   "Bluetooth address (e.g. AA:BB:CC:DD:EE:FF)");
    connect_args.end = arg_end(1);
    esp_console_cmd_t cmd_connect_cfg = {
        .command = "connect",
        .help = "Connect to a BLE peripheral by address",
        .func = cmd_connect,
        .argtable = &connect_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_connect_cfg));

    /* disconnect command */
    disconnect_args.addr_or_name = arg_str1(NULL, NULL, "<address|name>",
                                            "Connected device address or name");
    disconnect_args.end = arg_end(1);
    esp_console_cmd_t cmd_disconnect_cfg = {
        .command = "disconnect",
        .help = "Disconnect from a BLE peripheral by address or name",
        .func = cmd_disconnect,
        .argtable = &disconnect_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_disconnect_cfg));

    /* get_state command */
    esp_console_cmd_t cmd_get_state_cfg = {
        .command = "get_state",
        .help = "Get current GAP state and connection information",
        .func = cmd_get_state,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_get_state_cfg));

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
