#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "uart_transport.h"
#include "esp_transport.h"
#include "router.h"
#include "wifi.h"
#include "cam.h"
#include "websocket.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("SPI", ESP_LOG_INFO);
    esp_log_level_set("UART", ESP_LOG_INFO);
    esp_log_level_set("SYS", ESP_LOG_INFO);
    esp_log_level_set("ROUTER", ESP_LOG_INFO);
    esp_log_level_set("COM", ESP_LOG_INFO);
    esp_log_level_set("TEST", ESP_LOG_INFO);
    esp_log_level_set("WIFI", ESP_LOG_INFO);
    esp_log_level_set("ESP", ESP_LOG_INFO);
    esp_log_level_set("CAM", ESP_LOG_INFO);
    esp_log_level_set("camera", ESP_LOG_INFO);
    esp_log_level_set("cam_hal", ESP_LOG_INFO);
    esp_log_level_set("WEBSOCKET", ESP_LOG_INFO);
    // esp_log_level_set("lwip", ESP_LOG_DEBUG);

    ESP_LOGI("SYS", "\n\n -- Starting up --\n");

    xTaskCreate(uart_transport_init, "UART transport init", 8192, NULL, 2, NULL);
    xTaskCreate(espTransportInit, "ESP transport init", 8192, NULL, 2, NULL);
    xTaskCreate(wifi_init, "Router init", 8192, NULL, 2, NULL);
    xTaskCreate(router_init, "Router init", 8192, NULL, 2, NULL);
    xTaskCreate(camera_init, "Cam init", 8192, NULL, 2, NULL);
    xTaskCreate(socket_init, "Websocket init", 8192, NULL, 2, NULL);
}
