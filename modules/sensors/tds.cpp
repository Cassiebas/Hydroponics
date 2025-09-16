#include "tds.hpp"
#include "esp_log.h"

static const char* TAG = "tds";

// Sensor constants (Seeed datasheet)
// const float CALIBRATION_FACTOR_C = 0.946f;  // C constant
const float EC_TO_TDS_FACTOR_K = 1.0f;      // K factor
const float EC_SCALING_FACTOR = 10000.0f;   // datasheet (0-200000 µS/cm)

// Temperature compensation
const float TEMP_COMP_COEFF = 0.02f; // 2% per °C deviation
const float REF_TEMP = 25.0f;       // Reference temperature

// 2-point calibration constants
const float TDS_CAL_M = 1.725f;   
const float TDS_CAL_B = -967.5f;   

TDS::TDS(Adc& adc, const AdcConfig& config, size_t channel_idx, const std::vector<SensorData>& sensor_data)
    : Sensor(SensorData::Type::TDS), adc_(adc), config_(config), channel_idx_(channel_idx), sensor_data_(sensor_data) {
    ESP_LOGI(TAG, "TDS Sensor initialized on ADC channel %d", config_.channel);
}

esp_err_t TDS::read(float& value) {
    float voltage;
    esp_err_t ret = adc_.read(channel_idx_, voltage);

    if (ret == ESP_OK) {
        // 1) Voltage → EC
        float ppm = EC_SCALING_FACTOR * voltage;

        // 2) EC → TDS
        value = EC_TO_TDS_FACTOR_K * ppm;

        // Logging
        ESP_LOGD(TAG, "TDS Sensor: Voltage=%.3fV,  TDS=%.1f",
                 voltage, value);
    } else {
        ESP_LOGW(TAG, "TDS Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }

    return ret;
}
