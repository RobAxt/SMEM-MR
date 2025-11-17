idf_component_register(SRCS "SMEM-MR.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer nvs_flash driver switch_driver zigbee_enddevice)