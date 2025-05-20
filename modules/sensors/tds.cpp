#include "tds.hpp"
#include "esp_log.h"

static const char* TAG = "tds";

TDS::TDS(Adc& adc, const AdcConfig& config, size_t channel_idx)
    : Sensor(SensorData::Type::TDS), adc_(adc), config_(config), channel_idx_(channel_idx) {
    ESP_LOGI(TAG, "TDS Sensor initialized on ADC channel %d", config_.channel);
}

esp_err_t TDS::read(float& value) {
    float voltage;
    esp_err_t ret = adc_.read(channel_idx_, voltage, value);
    if (ret == ESP_OK) {
        value = voltage * 1667.0f; // TDS calculation
        ESP_LOGD(TAG, "TDS Sensor: Voltage=%.3fV, Value=%.0f ppm", voltage, value);
    } else {
        ESP_LOGW(TAG, "TDS Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }
    return ret;
}