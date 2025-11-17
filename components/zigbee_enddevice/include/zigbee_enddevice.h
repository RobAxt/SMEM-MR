/*
 * zigbee_enddevice.h
 *
 * Componente ESP-IDF para un Zigbee End Device.
 *
 * Proporciona dos funciones públicas:
 *  - void zigbee_endDevice_start(void);
 *      Inicializa y arranca el stack Zigbee (crea tareas internas).
 *
 *  - esp_err_t zigbee_endDevice_UpdateData(uint8_t new_value);
 *      Actualiza el valor de 8 bits que se expone en el cluster manufacturer-specific
 *      (intenta actualizar el atributo ZCL si el dispositivo ya está unido).
 *
 * Uso:
 *  - En app_main() inicializar NVS (nvs_flash_init) y luego llamar a zigbee_endDevice_start().
 *  - Si se quiere forzar un valor concreto desde la aplicación, llamar zigbee_endDevice_UpdateData().
 *
 */

#ifndef _ZIGBEE_ENDDEVICE_H_
#define _ZIGBEE_ENDDEVICE_H_

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Inicializa y arranca el Zigbee End Device.
 *
 * Crea internamente las tareas necesarias y registra los endpoints / clusters.
 * Debe llamarse después de inicializar NVS (nvs_flash_init) desde app_main().
 */
void zigbee_endDevice_start(void);

/**
 * @brief Actualiza el valor de 8 bits que se expone en el cluster custom (0xFC00).
 *
 * Si el dispositivo ya está unido, intenta también actualizar el atributo en el stack Zigbee.
 *
 * @param new_value Valor a escribir (0-255).
 * @return ESP_OK si el valor fue actualizado correctamente (o se almacenó localmente),
 *         ESP_FAIL si hubo error al actualizar el atributo en el stack.
 */
esp_err_t zigbee_endDevice_UpdateData(uint8_t new_value);

#endif // _ZIGBEE_ENDDEVICE_H_