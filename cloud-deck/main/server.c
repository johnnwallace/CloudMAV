#include "server.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "SERVER";

static EventGroupHandle_t startUpEventGroup;

static const int START_UP_SERVER_RUNNING = ( 1 << 0 );

// GET handler for URI "/"
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    const char* resp_str = "Hello World!\n";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// URI handler structure for GET /
static httpd_uri_t hello_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    .user_ctx  = NULL
};

// Function to start the HTTP server
static void start_webserver(void*)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello_uri);
    } else {
        ESP_LOGE(TAG, "Error starting server!");
    }

    xEventGroupSetBits(startUpEventGroup, START_UP_SERVER_RUNNING);

    vTaskDelete(NULL);
}

void init_webserver(void*)
{
    startUpEventGroup = xEventGroupCreate();

    ESP_LOGI(TAG, "Waiting to start");
    xTaskCreate(start_webserver, "Webserver", 5000, NULL, 1, NULL);
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_SERVER_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized");

    vTaskDelete(NULL);
}