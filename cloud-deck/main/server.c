#include "server.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"

#include "cpx.h"

/* TODO: Tune this for latency */
#define SERVER_TRANSPORT_MTU 1022
#define RX_QUEUE_LENGTH 4

static QueueHandle_t serverTxQueue;
static QueueHandle_t serverRxQueue;

static EventGroupHandle_t startUpEventGroup;

static CPXRoutablePacket_t txp;

static const int START_UP_SERVER_RUNNING = ( 1 << 0 );

static const char *TAG = "SERVER";
static const char *KEY = "cloudmav";

/*
 * Packet should look like this:
 * {
 *   "key": "xxxxx",
 *   "route": {
 *     "source": x,
 *     "destination": x,
 *     "function": x
 *   },
 *   "data": "xxxxx"
 * }
 */
enum JSONError{
    GOOD,
    BAD_PACKET,
    BAD_KEY,
    BAD_ROUTE
};

/* 
 * This only verifies that the JSON HAS the correct fields, not that the
 * fields are the correct type.
 */
static enum JSONError check_json(cJSON *json)
{
    /* Verify correct fields */
    if (!(cJSON_HasObjectItem(json, "key") &&
          cJSON_HasObjectItem(json, "route") &&
          cJSON_HasObjectItem(json, "data")))
    {
        return BAD_PACKET;
    }
    /* Verify correct key */
    cJSON *key = cJSON_GetObjectItem(json, "key");
    if (!cJSON_IsString(key))
    {
        return BAD_KEY;
    }
    if (strcmp(key->valuestring, KEY) != 0)
    {
        return BAD_KEY;
    }

    /* Verify route fields */
    cJSON *route = cJSON_GetObjectItem(json, "route");
    if (!(cJSON_HasObjectItem(route, "source") &&
          cJSON_HasObjectItem(route, "destination") &&
          cJSON_HasObjectItem(route, "function")))
    {
        return BAD_ROUTE;
    }

    return GOOD;
}

static esp_err_t return_error(httpd_req_t *req, cJSON *resp_json, int status, const char *message)
{
    cJSON_AddStringToObject(resp_json, "status", message);
    httpd_resp_send_err(req, status, cJSON_Print(resp_json));
    return ESP_FAIL;
}

// GET handler for URI "/"
static esp_err_t health_get_handler(httpd_req_t *req)
{
    const char *json_response = "{\"status\": \"ok\"}";

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// POST handler for URI "/control"
static esp_err_t control_post_handler(httpd_req_t *req)
{
    cJSON *resp_json = cJSON_CreateObject();
    cJSON *packet_json = cJSON_CreateObject();
    httpd_resp_set_type(req, "application/json");

    // Determine the total length of the incoming data
    int total_len = req->content_len;
    if (total_len <= 0) {
        ESP_LOGE(TAG, "No packet received");
        return return_error(req, resp_json, HTTPD_400_BAD_REQUEST, "No packet received");
    }

    // Allocate a buffer to hold the incoming data.
    // Adjust the size according to your expected packet size.
    char *buf = malloc(total_len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory for request buffer");
        return return_error(req, resp_json, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to allocate memory for request buffer");
    }

    // Receive the data in a loop in case it arrives in chunks
    int received = 0, ret;
    while (received < total_len) {
        ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            free(buf);
            ESP_LOGE(TAG, "Error receiving data: %d", ret);
            return return_error(req, resp_json, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive post data");
        }
        received += ret;
    }
    buf[total_len] = '\0';  // Null-terminate the buffer

    ESP_LOGI(TAG, "Received packet: %s", buf);

    packet_json = cJSON_Parse(buf);
    enum JSONError json_result = check_json(packet_json);
    if (json_result != GOOD)
    {
        cJSON_AddNumberToObject(resp_json, "error", json_result);
        return return_error(req, resp_json, (json_result == BAD_KEY) ? HTTPD_403_FORBIDDEN : HTTPD_400_BAD_REQUEST, "Bad JSON structure");
    }

    // Construct a CPXRoutablePacket_t from the received json
    cJSON *route = cJSON_GetObjectItem(packet_json, "route");
    if (!cJSON_IsObject(route))
    {
        return return_error(req, resp_json, HTTPD_400_BAD_REQUEST, "Bad route");
    }

    cJSON *source = cJSON_GetObjectItem(route, "source");
    cJSON *destination = cJSON_GetObjectItem(route, "destination");
    cJSON *function = cJSON_GetObjectItem(route, "function");
    cJSON *data = cJSON_GetObjectItem(packet_json, "data");

    if (!cJSON_IsNumber(source) || !cJSON_IsNumber(destination) || !cJSON_IsNumber(function) || !cJSON_IsString(data))
    {
        return return_error(req, resp_json, HTTPD_400_BAD_REQUEST, "Bad route");
    }
    cpxInitRoute(source->valueint, destination->valueint,
                 function->valueint, &txp.route);
    memcpy(txp.data, data->valuestring, strlen(data->valuestring) + 1);
    txp.dataLength = strlen(data->valuestring) + 1;

    xQueueSend(serverTxQueue, &txp, portMAX_DELAY);

    // Send a response back to the client.
    cJSON_AddStringToObject(resp_json, "status", "Received post data");
    cJSON_AddStringToObject(resp_json, "packet", buf);
    httpd_resp_send(req, cJSON_Print(resp_json), HTTPD_RESP_USE_STRLEN);

    free(buf);
    return ESP_OK;
}

// URI handler structure for GET /
static httpd_uri_t health_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = health_get_handler,
    .user_ctx  = NULL
};

// URI handler structure for POST /control
static httpd_uri_t control_uri = {
    .uri       = "/control",
    .method    = HTTP_POST,
    .handler   = control_post_handler,
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
        httpd_register_uri_handler(server, &health_uri);
        httpd_register_uri_handler(server, &control_uri);
    } else {
        ESP_LOGE(TAG, "Error starting server!");
    }

    xEventGroupSetBits(startUpEventGroup, START_UP_SERVER_RUNNING);

    vTaskDelete(NULL);
}

void init_webserver(void*)
{
    startUpEventGroup = xEventGroupCreate();

    serverTxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    serverRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

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

void serverTransportSend(const CPXRoutablePacket_t* packet) {
    assert(packet->dataLength <= SERVER_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
    xQueueSend(serverRxQueue, packet, portMAX_DELAY);
}

void serverTransportReceive(CPXRoutablePacket_t* packet) {
    ESP_LOGI(TAG, "Received packet");
    xQueueReceive(serverTxQueue, packet, portMAX_DELAY);
    packet->route.lastPacket = true;
}