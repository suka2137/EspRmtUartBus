idf_component_register(SRCS "src/rmt-uart.c" "src/ub.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_rmt vfs
                    )
