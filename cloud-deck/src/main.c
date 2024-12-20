#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "cpx.h"
#include "router.h"
#include "uart_transport.h"
#include "mcu_transport.h"

# define RESET_PIN 2

void app_main() {
    xTaskCreate(uartTransportInit, "UART transport init", 128, NULL, 2, NULL);
    xTaskCreate(espTransportInit, "ESP transport init", 128, NULL, 2, NULL);
    xTaskCreate(router_init, "Router init", 128, NULL, 2, NULL);

    vTaskStartScheduler();
}

void loop() {
}