/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef LED_H
#define LED_H

/* Includes */
/* ESP APIs */
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"

/* Defines */
#define BLINK_GPIO CONFIG_BLINK_GPIO

/* Public function declarations */
uint8_t get_led_state(void);
void led_on(uint32_t green, uint32_t red, uint32_t blue);
void led_off(void);
void led_init(void);

#endif // LED_H
