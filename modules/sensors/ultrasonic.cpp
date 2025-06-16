#include "ultrasonic.hpp"
#include "esp_log.h"

static const char* TAG = "ultrasonic";

Ultrasonic::Ultrasonic(Uart& uart)
    : Sensor(SensorData::Type::WATER_LEVEL), uart_(uart) {
    ESP_LOGI(TAG, "Ultrasonic Sensor initialized on UART");
}

esp_err_t Ultrasonic::read(float& value) {
    if (uart_.read_sen0311_distance(value)) {
        ESP_LOGD(TAG, "Ultrasonic Sensor: Distance=%.1f cm", value);
        return ESP_OK;
    } else {
        // ESP_LOGW(TAG, "Ultrasonic Sensor read failed");
        value = 0.0f;
        return ESP_FAIL;
    }
}