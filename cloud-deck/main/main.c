/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "uart_transport.h"
#include "esp_transport.h"
#include "router.h"

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("SPI", ESP_LOG_INFO);
    esp_log_level_set("UART", ESP_LOG_INFO);
    esp_log_level_set("SYS", ESP_LOG_INFO);
    esp_log_level_set("ROUTER", ESP_LOG_INFO);
    esp_log_level_set("COM", ESP_LOG_INFO);
    esp_log_level_set("TEST", ESP_LOG_INFO);
    esp_log_level_set("WIFI", ESP_LOG_INFO);

    ESP_LOGI("SYS", "\n\n -- Starting up --\n");

    xTaskCreate(uart_transport_init, "UART transport init", 8192, NULL, 2, NULL);
    xTaskCreate(espTransportInit, "ESP transport init", 8192, NULL, 2, NULL);
    xTaskCreate(router_init, "Router init", 8192, NULL, 2, NULL);
}
