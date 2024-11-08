#include <arduino_freertos.h>
#include <queue.h>

#include "cpx.h"
#include "router.h"
#include "esp_transport.h"
#include "uart_transport.h"

static void blink(void*) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

FLASHMEM __attribute__((noinline))void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("Starting deck test");

    router_init();
    espTransportInit();
    uartTransportInit();

    xTaskCreate(blink, "Blink", 128, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void loop() {
}