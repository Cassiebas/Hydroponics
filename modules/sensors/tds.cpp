#include "tds.hpp"
#include "esp_log.h"

static const char* TAG = "tds";

// Calibration constants for the SEED EC probe
const float CALIBRATION_FACTOR_C = 0.946f;  // Provided C value
const float EC_TO_TDS_FACTOR_K = 1.0f;      // Provided K value
const float EC_SCALING_FACTOR = 10000.0f;    // From datasheet (0-200000 µS/cm range: EC = 1000 * VOLTAGE)

// Temperature compensation coefficient (2% per °C deviation from 25°C)
const float TEMP_COMP_COEFF = 0.02f;
const float REF_TEMP = 25.0f;

TDS::TDS(Adc& adc, const AdcConfig& config, size_t channel_idx, const std::vector<SensorData>& sensor_data)
    : Sensor(SensorData::Type::TDS), adc_(adc), config_(config), channel_idx_(channel_idx), sensor_data_(sensor_data) {
    ESP_LOGI(TAG, "TDS Sensor initialized on ADC channel %d", config_.channel);
}

esp_err_t TDS::read(float& value) {
    float voltage;
    esp_err_t ret = adc_.read(channel_idx_, voltage);
    ESP_LOGI(TAG, "TDS Voltage: %.3fV", voltage);
    
    if (ret == ESP_OK) {
        // Step 1: Convert voltage to EC (µS/cm) using datasheet formula
        float ec_raw = EC_SCALING_FACTOR * voltage;  // EC = 1000 * VOLTAGE

        // Step 2: Apply calibration factor C
        float ec_calibrated = CALIBRATION_FACTOR_C * ec_raw;  // EC_calibrated = 0.946 * EC_raw

        // Step 3: Apply temperature compensation
        float temperature = 0.0f;
        for (const auto& data : sensor_data_) {
            if (data.type == SensorData::Type::NTC) {
                temperature = data.value;
                break;
            }
        }
        float ec_compensated = ec_calibrated / (1.0f + TEMP_COMP_COEFF * (temperature - REF_TEMP));

        // Step 4: Convert EC to TDS (ppm) using K factor
        value = EC_TO_TDS_FACTOR_K * ec_compensated;  // TDS = K * EC_compensated

    } else {
        ESP_LOGW(TAG, "TDS Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }
    return ret;
}