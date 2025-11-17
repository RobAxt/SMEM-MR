idf_component_register(SRCS "source/zigbee_enddevice.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash espressif__esp-zigbee-lib espressif__esp-zboss-lib)
