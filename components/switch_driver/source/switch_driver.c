#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_driver.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "ESP_ZB_SWITCH";

/* single shared queue and task */
static QueueHandle_t gpio_evt_queue = NULL;
static TaskHandle_t button_task_handle = NULL;

/* storage for configured pins and callbacks */
static switch_func_pair_t configured_pairs[SWITCH_DRIVER_MAX_PINS];
static esp_switch_callback_t configured_cbs[SWITCH_DRIVER_MAX_PINS];
static uint8_t configured_count = 0;

/* ISR arg indices (addresses passed to ISR) */
static uint8_t isr_arg_indices[SWITCH_DRIVER_MAX_PINS];
static volatile bool processing_flags[SWITCH_DRIVER_MAX_PINS]; /* dedup: true while pin is queued/being processed */
static bool isr_service_installed = false;

/* IRAM-safe ISR handler: attempt to mark pin as processing and enqueue its index.
   If the pin is already marked processing, the ISR returns immediately (dedup). */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint8_t idx = (uint8_t)(uintptr_t)arg;
    if (gpio_evt_queue == NULL) return;
    if (idx >= SWITCH_DRIVER_MAX_PINS) return;

    /* If already being processed, ignore further interrupts for this pin */
    if (processing_flags[idx]) {
        return;
    }

    /* Mark as processing first. If queue send fails, clear the flag. */
    processing_flags[idx] = true;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t sent = xQueueSendFromISR(gpio_evt_queue, &idx, &xHigherPriorityTaskWoken);
    if (sent == pdFALSE) {
        /* queue full or failed: clear flag and return */
        processing_flags[idx] = false;
        return;
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/*
 * Shared task: reads index from queue, performs per-pin debounce (both edges)
 * and calls the corresponding callback with the debounced level.
 *
 * Behavior:
 *  - ISR is configured for GPIO_INTR_ANYEDGE (both edges).
 *  - When ISR posts an index, the task disables interrupts for that pin only,
 *    samples the pin level several times to confirm stability and then calls the callback
 *    passing the debounced level. Finally re-enables the interrupt for that pin and clears the processing flag.
 *
 * Note: pull-ups are kept enabled in gpio_config() (pull_up_en = 1).
 */
static void switch_driver_button_detected(void *arg)
{
    (void)arg;
    uint8_t idx = 0xff;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &idx, portMAX_DELAY) == pdTRUE) {
            if (idx >= configured_count) {
                /* invalid index, clear processing just in case and continue */
                if (idx < SWITCH_DRIVER_MAX_PINS) processing_flags[idx] = false;
                continue;
            }

            /* Local copy of pair */
            switch_func_pair_t button_func_pair = configured_pairs[idx];
            gpio_num_t io_num = (gpio_num_t)button_func_pair.pin;

            /* Disable interrupt for this pin while debouncing */
            gpio_intr_disable(button_func_pair.pin);

            /* Debounce: sample consecutive reads and aim for stability */
            int last_level = -1;
            int stable_count = 0;
            const TickType_t delay_ticks = pdMS_TO_TICKS(SWITCH_DRIVER_DEBOUNCE_MS);

            for (int s = 0; s < SWITCH_DRIVER_DEBOUNCE_SAMPLES; ++s) {
                int lvl = gpio_get_level(io_num);
                if (s == 0) {
                    last_level = lvl;
                    stable_count = 1;
                } else {
                    if (lvl == last_level) {
                        stable_count++;
                    } else {
                        /* level changed during debounce: restart counting with new level */
                        last_level = lvl;
                        stable_count = 1;
                    }
                }
                vTaskDelay(delay_ticks);
            }

            /* Consider last_level as the debounced level */
            int debounced_level = last_level;

            /* Invoke callback with debounced level (if provided) */
            if (idx < configured_count && configured_cbs[idx]) {
                configured_cbs[idx](&button_func_pair, debounced_level);
            }

            ESP_LOGD(TAG, "Pin %d (idx %d) debounced level=%d", (int)button_func_pair.pin, (int)idx, (int)debounced_level);

            /* Re-enable interrupt only for this pin and clear processing flag */
            gpio_intr_enable(button_func_pair.pin);
            processing_flags[idx] = false;

            /* Small optional delay to avoid immediate re-entry from bounce */
            /* vTaskDelay(pdMS_TO_TICKS(1)); */
        }
    }
}

/* initialize the driver with a single shared task/queue and up to SWITCH_DRIVER_MAX_PINS ISRs/callbacks */
bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t *cbs)
{
    if (button_func_pair == NULL || button_num == 0 || button_num > SWITCH_DRIVER_MAX_PINS) {
        ESP_LOGE(TAG, "Invalid params; max supported pins = %d", (int)SWITCH_DRIVER_MAX_PINS);
        return false;
    }

    /* if already initialized, deinit first */
    if (gpio_evt_queue != NULL || button_task_handle != NULL) {
        ESP_LOGW(TAG, "Driver already initialized, deinitializing first");
        switch_driver_deinit();
    }

    /* copy configuration (so user can free/stack the input array) */
    memset(configured_pairs, 0, sizeof(configured_pairs));
    memset(configured_cbs, 0, sizeof(configured_cbs));
    memset(processing_flags, 0, sizeof(processing_flags));
    for (int i = 0; i < button_num; ++i) {
        configured_pairs[i] = button_func_pair[i];
        configured_cbs[i] = (cbs != NULL) ? cbs[i] : NULL;
        isr_arg_indices[i] = (uint8_t)i;
        processing_flags[i] = false;
    }
    configured_count = button_num;

    /* build pin bit mask for gpio_config; keep pull-ups enabled */
    gpio_config_t io_conf = {};
    uint64_t pin_bit_mask = 0;
    for (int i = 0; i < configured_count; ++i) {
        pin_bit_mask |= (1ULL << configured_pairs[i].pin);
    }
    /* Detect both edges */
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1; /* keep pull-ups enabled */
    gpio_config(&io_conf);

    /* create single shared queue (store uint8_t index) */
    gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Queue creation failed");
        configured_count = 0;
        return false;
    }

    /* install ISR service once */
    if (!isr_service_installed) {
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        isr_service_installed = true;
    }

    /* add ISR handlers for each configured pin (pass index as arg) */
    for (int i = 0; i < configured_count; ++i) {
        gpio_set_intr_type(configured_pairs[i].pin, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(configured_pairs[i].pin, gpio_isr_handler, (void *)(uintptr_t)isr_arg_indices[i]);
    }

    /* create the single shared task */
    xTaskCreate(switch_driver_button_detected, "button_detected", 4096, NULL, 10, &button_task_handle);
    if (button_task_handle == NULL) {
        ESP_LOGE(TAG, "Task creation failed");
        /* cleanup ISRs and queue */
        for (int i = 0; i < configured_count; ++i) {
            gpio_isr_handler_remove(configured_pairs[i].pin);
        }
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        configured_count = 0;
        return false;
    }

    ESP_LOGI(TAG, "Switch driver initialized: %d pins (single task/queue, dedup per-pin, both-edge callbacks)", (int)configured_count);

    /* Read initial state of all pins and call callbacks with initial values */
    ESP_LOGI(TAG, "Reading initial pin states and calling callbacks");
    for (int i = 0; i < configured_count; ++i) {
        int initial_level = gpio_get_level((gpio_num_t)configured_pairs[i].pin);
        ESP_LOGI(TAG, "Pin %d initial level: %d", (int)configured_pairs[i].pin, (int)initial_level);
        
        /* Call callback with initial state if callback is configured */
        if (configured_cbs[i]) {
            configured_cbs[i](&configured_pairs[i], initial_level);
        }
    }

    return true;
}

void switch_driver_deinit(void)
{
    if (configured_count == 0 && gpio_evt_queue == NULL && button_task_handle == NULL) {
        return;
    }

    /* remove ISRs */
    for (int i = 0; i < configured_count; ++i) {
        gpio_isr_handler_remove(configured_pairs[i].pin);
    }

    /* delete task */
    if (button_task_handle) {
        vTaskDelete(button_task_handle);
        button_task_handle = NULL;
    }

    /* delete queue */
    if (gpio_evt_queue) {
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
    }

    memset(configured_pairs, 0, sizeof(configured_pairs));
    memset(configured_cbs, 0, sizeof(configured_cbs));
    memset((void*)processing_flags, 0, sizeof(processing_flags));
    configured_count = 0;

    ESP_LOGI(TAG, "Switch driver deinitialized");
}