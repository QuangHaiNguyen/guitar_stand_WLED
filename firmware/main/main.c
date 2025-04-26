#include <stdio.h>
#include "esp_log.h"

static const char* TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Hello, ESP32!");
    ESP_LOGI(TAG, "This is a simple example of logging in ESP-IDF.");
    ESP_LOGI(TAG, "Make sure to configure the logging level in menuconfig.");
    ESP_LOGI(TAG, "You can use different log levels: DEBUG, INFO, WARN, ERROR, and FATAL.");
    ESP_LOGI(TAG, "This is a test message with a number: %d", 42);
    ESP_LOGI(TAG, "Logging is very useful for debugging and monitoring your application.");
    ESP_LOGI(TAG, "Goodbye!");
}