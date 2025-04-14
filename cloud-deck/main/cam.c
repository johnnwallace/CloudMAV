#include "cam.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_netif.h"

#include "websocket.h"

#define CONFIG_JPEG_QUALITY 32
#define CAMERA_QUEUE_LENGTH 4

static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk = 10,
    .pin_sccb_sda = 40,
    .pin_sccb_scl = 39,

    .pin_d7 = 48,
    .pin_d6 = 11,
    .pin_d5 = 12,
    .pin_d4 = 14,
    .pin_d3 = 16,
    .pin_d2 = 18,
    .pin_d1 = 17,
    .pin_d0 = 15,
    .pin_vsync = 38,
    .pin_href = 47,
    .pin_pclk = 13,

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    // .frame_size = FRAMESIZE_SVGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    .frame_size = FRAMESIZE_VGA,

    .jpeg_quality = CONFIG_JPEG_QUALITY, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = CAMERA_QUEUE_LENGTH, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_LATEST,//CAMERA_GRAB_LATEST. Sets when buffers should be filled

};

#define FREQ 30

static const char *TAG = "CAM";

static const int START_UP_STREAMING_TASK = BIT1;
static EventGroupHandle_t startUpEventGroup;

static void streaming_task(void *pvParameters)
{
    xEventGroupSetBits(startUpEventGroup, START_UP_STREAMING_TASK);

    while(1) {
        xEventGroupWaitBits(wsConnectionEventGroup, WS_CONNECTION_ACTIVE,
                            pdFALSE,
                            pdTRUE,
                            portMAX_DELAY);
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Failed to capture frame");
            continue;
        }
        
        esp_err_t ret = ws_send_image(fb);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send image: %d", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void camera_init()
{
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_STREAMING_TASK);
    #if CONFIG_CAMERA_CORE0
        xTaskCreatePinnedToCore(streaming_task, "Cam WiFi TX", 5000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    #elif CONFIG_CAMERA_CORE1
        xTaskCreatePinnedToCore(streaming_task, "Cam WiFi TX", 5000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    #endif
    ESP_LOGI(TAG, "Waiting for cam wifi streaming to start");
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_STREAMING_TASK,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Streaming initialized");

    vTaskDelete(NULL);
}