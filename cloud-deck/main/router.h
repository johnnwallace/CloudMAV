#pragma once

#include "cpx.h"

#include "esp_transport.h"
#include "uart_transport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

void router_init(void*);