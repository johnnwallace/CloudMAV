idf_component_register(SRCS "websocket.c" "cam.c" "wifi.c" "main.c" "uart_transport.c" "router.c" "cpx.c" "esp_transport.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES esp_https_server esp_driver_uart esp_driver_gpio esp_wifi nvs_flash esp_system esp_driver_cam)

target_add_binary_data(${COMPONENT_TARGET} "../certificates/servercert.pem" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "../certificates/serverkey.pem" TEXT)