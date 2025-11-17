/*
 * zigbee_enddevice.c
 *
 * Implementación del componente Zigbee End Device basada en el main.c proporcionado.
 *
 * - Expone zigbee_endDevice_start() para inicializar/arrancar el dispositivo.
 * - Expone zigbee_endDevice_UpdateData() para forzar la actualización del valor uint8_t.
 *
 * Internamente implementa:
 * - esp_zb_app_signal_handler() (necesario para que el stack Zigbee notifique señales)
 * - creación de clusters y endpoints (incluyendo cluster manufacturer-specific 0xFC00)
 * - tarea para arrancar el stack Zigbee y tarea periódica que actualiza el atributo.
 *
 * Notas:
 * - Este componente asume que la aplicación principal ya ejecutó nvs_flash_init().
 * - El cluster custom usa attribute ID 0x0000 y tipo uint8.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "esp_zigbee_core.h"
#include "zigbee_enddevice.h"

static const char *TAG = "zb_end_comp";

/* Configuración (copiada del main original) */
#define SRC_ENDPOINT 1
#define CLUSTER_ID 0xFC00
#define SEND_INTERVAL_MS 15000

/* Valor local del atributo (compartido entre tareas y API) */
static uint8_t s_value = 0;

/* Semaphore para proteger acceso a s_value */
static SemaphoreHandle_t s_value_lock = NULL;

/* Indicador de unión a la red */
static bool device_joined = false;

/* Forward declarations de funciones internas */
static esp_zb_cluster_list_t *custom_clusters_create(void);
static esp_zb_ep_list_t *custom_ep_list_create(void);
static void zigbee_init_and_start(void);
static void esp_zb_task(void *pvParameters);

/* --------------------------------------------------------------------------
 * Funciones públicas
 * -------------------------------------------------------------------------- */

void zigbee_endDevice_start(void)
{
    if (s_value_lock == NULL) {
        s_value_lock = xSemaphoreCreateMutex();
        if (s_value_lock == NULL) {
            ESP_LOGE(TAG, "Failed to create value mutex");
            return;
        }
    }

    ESP_LOGI(TAG, "Starting Zigbee End Device component");

    /* Crear tarea Zigbee (stack) y tarea de envío/actualización de atributo */
    xTaskCreate(esp_zb_task, "esp_zb_task", 8192, NULL, 5, NULL);
}

/**
 * Actualiza el valor interno y, si el dispositivo ya está unido, intenta actualizar el atributo
 * en el stack Zigbee (esp_zb_zcl_set_attribute_val).
 */
esp_err_t zigbee_endDevice_UpdateData(uint8_t new_value)
{
    esp_err_t ret = ESP_OK;

    if (s_value_lock) xSemaphoreTake(s_value_lock, portMAX_DELAY);
    s_value = new_value;
    if (s_value_lock) xSemaphoreGive(s_value_lock);

    if (!device_joined) {
        ESP_LOGW(TAG, "Device not joined yet - value updated locally: 0x%02X", (int)new_value);
        return ESP_OK;
    }

    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        SRC_ENDPOINT,
        CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        0x0000,
        &s_value,
        false
    );

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGW(TAG, "Set attribute failed when forcing update: %d", (int)status);
        ret = ESP_FAIL;
    }
    else
        ESP_LOGI(TAG, "Value forced to 0x%02X and attribute updated in stack", (int)new_value);

    return ret;
}

/* --------------------------------------------------------------------------
 * Implementación del handler de señales Zigbee
 * -------------------------------------------------------------------------- */

/* El stack Zigbee llama a esta función globalmente */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
            ESP_LOGW(TAG, "Erasing Zigbee NVRAM and restarting...");
            esp_zb_nvram_erase_at_start(true);
            esp_restart();
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());

            device_joined = true;
            
            /* Send current value to coordinator after joining */
            uint8_t current_value;
            if (s_value_lock) xSemaphoreTake(s_value_lock, portMAX_DELAY);
            current_value = s_value;
            if (s_value_lock) xSemaphoreGive(s_value_lock);
            
            ESP_LOGI(TAG, "Sending initial value to coordinator: 0x%02X", (int)current_value);
            zigbee_endDevice_UpdateData(current_value);
        } else {
            ESP_LOGW(TAG, "Network steering was not successful (status: %d). Retrying in 5 seconds...", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;

    default:
        ESP_LOGI(TAG, "Unhandled ZDO signal: %d, status: %d", sig_type, err_status);
        break;
    }
}

/* --------------------------------------------------------------------------
 * Creación de clusters y endpoints (copia adaptada del main.c original)
 * -------------------------------------------------------------------------- */

static esp_zb_cluster_list_t *custom_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster */
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Custom manufacturer-specific cluster 0xFC00 */
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID);

    /* Attribute ID 0x0000 - uint8_t */
    esp_zb_custom_cluster_add_custom_attr(
        custom_cluster,
        0x0000,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
        &s_value
    );

    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cluster_list;
}

static esp_zb_ep_list_t *custom_ep_list_create(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = SRC_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_clusters_create(), endpoint_config);
    return ep_list;
}

/* --------------------------------------------------------------------------
 * Inicialización del stack Zigbee y tareas
 * -------------------------------------------------------------------------- */

static void zigbee_init_and_start(void)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,
            },
        },
    };

    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = custom_ep_list_create();
    esp_zb_device_register(ep_list);

    /* Scan all channels */
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_LOGI(TAG, "Initializing Zigbee End Device (component)...");
    ESP_ERROR_CHECK(esp_zb_start(false));
}

static void esp_zb_task(void *pvParameters)
{
    zigbee_init_and_start();

    /* Ejecutar iteración principal del loop Zigbee (no retorna) */
    esp_zb_main_loop_iteration();

    /* Si llega aquí, eliminar tarea */
    vTaskDelete(NULL);
}
