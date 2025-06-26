#include "ntc.hpp"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "ntc_sensor";

NTC::NTC(Adc& adc, const AdcConfig& config, size_t channel_idx)
    : Sensor(SensorData::Type::NTC), adc_(adc), config_(config), channel_idx_(channel_idx) {
    ESP_LOGI(TAG, "NTC Sensor initialized on ADC channel %d", config_.channel);
}

esp_err_t NTC::read(float& value) {
    float voltage;
    esp_err_t ret = adc_.read(channel_idx_, voltage);
    if (ret == ESP_OK) {
        const float R_REF = 10000.0f;
        if (voltage < 0.01f || voltage > 3.29f) {
            ESP_LOGW(TAG, "NTC Sensor invalid voltage: %.3fV", voltage);
            value = 0.0f;
            return ESP_ERR_INVALID_STATE;
        }
        float r_ntc = R_REF * ((3.3f / voltage) - 1.0f);
        const float A = 0.8999648402e-3f;
        const float B = 2.494581846e-4f;
        const float C = 2.002476456e-7f;
        float ln_r = log(r_ntc);
        float ln_r_cube = ln_r * ln_r * ln_r;
        float temp_k = 1.0f / (A + B * ln_r + C * ln_r_cube);
        value = temp_k - 273.15f; // Convert to Celsius
        ESP_LOGD(TAG, "NTC Sensor: Voltage=%.3fV, Resistance=%.0fΩ, Temperature=%.2f°C", voltage, r_ntc, value);
    } else {
        ESP_LOGW(TAG, "NTC Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }
    return ret;
}