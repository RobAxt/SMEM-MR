/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * 4. Any software provided in binary form under this license must not be reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* config button level depends on the pull up/down setting
   push button level is on level = 1 when pull-down enable
   push button level is on level = 0 when pull-up enable
*/
#ifndef GPIO_INPUT_LEVEL_ON
#define GPIO_INPUT_LEVEL_ON     0
#endif

#ifndef ESP_INTR_FLAG_DEFAULT
#define ESP_INTR_FLAG_DEFAULT 0
#endif

/* Maximum number of pins (ISRs + callbacks) supported by the single shared task/queue */
#ifndef SWITCH_DRIVER_MAX_PINS
#define SWITCH_DRIVER_MAX_PINS 4
#endif

/* Debounce tuning (samples and ms between samples) */
#ifndef SWITCH_DRIVER_DEBOUNCE_SAMPLES
#define SWITCH_DRIVER_DEBOUNCE_SAMPLES 3
#endif
#ifndef SWITCH_DRIVER_DEBOUNCE_MS
#define SWITCH_DRIVER_DEBOUNCE_MS 50
#endif

#define PAIR_SIZE(TYPE_STR_PAIR) (sizeof(TYPE_STR_PAIR) / sizeof(TYPE_STR_PAIR[0]))

typedef enum {
    SWITCH_IDLE,
    SWITCH_PRESS_ARMED,
    SWITCH_PRESS_DETECTED,
    SWITCH_PRESSED,
    SWITCH_RELEASE_DETECTED,
} switch_state_t;

typedef enum {
    SWITCH_ON_CONTROL,
    SWITCH_OFF_CONTROL,
    SWITCH_ONOFF_TOGGLE_CONTROL,
    SWITCH_LEVEL_UP_CONTROL,
    SWITCH_LEVEL_DOWN_CONTROL,
    SWITCH_LEVEL_CYCLE_CONTROL,
    SWITCH_COLOR_CONTROL,
} switch_func_t;

typedef struct {
    uint32_t pin;
    switch_func_t func;
} switch_func_pair_t;

/* Callback receives pair and current debounced level (0 or 1). */
typedef void (*esp_switch_callback_t)(switch_func_pair_t *param, int level);

/**
 * @brief init function for switch and callback setup (single shared task/queue).
 *
 * @param button_func_pair      pointer to an array of switch_func_pair_t for each pin.
 * @param button_num            number of entries in button_func_pair (must be <= SWITCH_DRIVER_MAX_PINS).
 * @param cbs                   array of callbacks, one per pin (length == button_num). Each may be NULL.
 *
 * @return true on success, false on failure.
 */
bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t *cbs);

/**
 * @brief Deinitialize the switch driver: remove ISRs and free internal resources.
 *
 * After calling this, you can call switch_driver_init() again.
 */
void switch_driver_deinit(void);

#ifdef __cplusplus
} // extern "C"
#endif