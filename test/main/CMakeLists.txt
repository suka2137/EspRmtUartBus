idf_component_register(SRCS "test_rmt_uart.c" "test_bus.c"
                       INCLUDE_DIRS "." "../../include" "../../src"
                       PRIV_REQUIRES unity esp_driver_gpio esp_driver_rmt esp_driver_uart esp_timer vfs)
