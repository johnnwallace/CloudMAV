#include <unity.h>

#include <arduino_freertos.h>
#include <event_groups.h>

#define TEST_BIT (1 << 0)

static EventGroupHandle_t evGroup;

void task_wait_for_event_bits() {
    xEventGroupWaitBits(evGroup,
                        TEST_BIT,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);
    Serial.println("Test bit received!");
}

void task_set_event_bits() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    xEventGroupSetBits(evGroup, TEST_BIT);
}

void test_tasks() {
    evGroup = xEventGroupCreate();
    xEventGroupClearBits(evGroup, TEST_BIT);
    xTaskCreate(task_wait_for_event_bits, "Wait for event bits", 5000, NULL, 1, NULL);
    xTaskCreate(task_set_event_bits, "Set event bits", 5000, NULL, 1, NULL);
}

void test() {
    TEST_ASSERT_TRUE(true);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_tasks);
    UNITY_END();
}