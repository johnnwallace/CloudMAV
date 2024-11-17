#include <arduino_freertos.h>
#include <queue.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "cpx.h"
#include "router.h"
#include "esp_transport.h"
#include "uart_transport.h"

# define RESET_PIN 2

void reset() {
    SCB_AIRCR = 0x05FA0004;
}

static void blink(void*) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(400));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

FLASHMEM __attribute__((noinline))void setup() {
    pinMode(RESET_PIN, arduino::INPUT_PULLUP); 
    // attachInterrupt(digitalPinToInterrupt(RESET_PIN), reset, arduino::LOW);

    Serial.begin(9600);
    // delay(1000);

    xTaskCreate(uartTransportInit, "UART transport init", 128, nullptr, 2, nullptr);
    xTaskCreate(espTransportInit, "ESP transport init", 128, nullptr, 2, nullptr);
    xTaskCreate(router_init, "Router init", 128, nullptr, 2, nullptr);

    xTaskCreate(blink, "Blink", 128, nullptr, 2, nullptr);

    vTaskStartScheduler();
    Serial.println("Starting deck test");
}

void loop() {
}