idf_component_register(SRCS "main.c" "uart_transport.c" "router.c" "cpx.c" "esp_transport.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES esp_driver_uart esp_driver_gpio)