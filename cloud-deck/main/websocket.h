#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_camera.h"

#define WS_CONNECTION_ACTIVE (1 << 0)

extern EventGroupHandle_t wsConnectionEventGroup;

void ws_send_image(const camera_fb_t* im);
void socket_init();