#include <stdio.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "switch_driver.h"
#include "zigbee_enddevice.h"

#define PIN0 (1<<0)
#define PIN1 (1<<1)
#define PIN2 (1<<2)
#define PIN3 (1<<3)

static const char *TAG = "app_main";
static uint8_t pin_state = 0;
static uint8_t prev_pin_state = 0;

/* Example callbacks for pins 4,5,6,7 - now receive debounced level */
static void switch_cb_pin4(switch_func_pair_t *param, int level)
{
    ESP_LOGI(TAG, "PIN4 callback: pin=%d func=%d level=%d", (int)param->pin, (int)param->func, level);
    if (level == GPIO_INPUT_LEVEL_ON) 
        pin_state |= PIN0;
    else
        pin_state &= ~PIN0;
}

static void switch_cb_pin5(switch_func_pair_t *param, int level)
{
    ESP_LOGI(TAG, "PIN5 callback: pin=%d func=%d level=%d", (int)param->pin, (int)param->func, level);
    if (level == GPIO_INPUT_LEVEL_ON) 
        pin_state |= PIN1;
    else
        pin_state &= ~PIN1;
}

static void switch_cb_pin6(switch_func_pair_t *param, int level)
{
    ESP_LOGI(TAG, "PIN6 callback: pin=%d func=%d level=%d", (int)param->pin, (int)param->func, level);
    if (level == GPIO_INPUT_LEVEL_ON) 
        pin_state |= PIN2;
    else
        pin_state &= ~PIN2;
}

static void switch_cb_pin7(switch_func_pair_t *param, int level)
{
    ESP_LOGI(TAG, "PIN7 callback: pin=%d func=%d level=%d", (int)param->pin, (int)param->func, level);
    if (level == GPIO_INPUT_LEVEL_ON) 
        pin_state |= PIN3;
    else
        pin_state &= ~PIN3;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing SMEM-MR application");

    // Inicializar NVS (Non-Volatile Storage) para configuración persistente
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Define pairs for pins 4,5,6,7 */
    switch_func_pair_t pairs[] = {
        { .pin = GPIO_NUM_4, .func = SWITCH_ONOFF_TOGGLE_CONTROL },
        { .pin = GPIO_NUM_5, .func = SWITCH_ONOFF_TOGGLE_CONTROL },
        { .pin = GPIO_NUM_6, .func = SWITCH_ONOFF_TOGGLE_CONTROL },
        { .pin = GPIO_NUM_7, .func = SWITCH_ONOFF_TOGGLE_CONTROL },
    };

    /* Array of callbacks per pin */
    esp_switch_callback_t cbs[] = {
        switch_cb_pin4,
        switch_cb_pin5,
        switch_cb_pin6,
        switch_cb_pin7,
    };

    if (!switch_driver_init(pairs, (uint8_t)(sizeof(pairs)/sizeof(pairs[0])), cbs)) 
        ESP_LOGE(TAG, "switch_driver_init failed");
    else 
        ESP_LOGI(TAG, "switch_driver initialized for pins 4,5,6,7");
    
    /* Start Zigbee end device */
    zigbee_endDevice_start();

    while (1) {
        ESP_LOGD(TAG, "Running main loop");
        
        if(pin_state != prev_pin_state) 
        {
            ESP_LOGI(TAG, "Pin state changed: 0x%02X -> 0x%02X", prev_pin_state, pin_state);
            zigbee_endDevice_UpdateData(pin_state);
            prev_pin_state = pin_state;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}