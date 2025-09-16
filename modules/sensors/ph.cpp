#include "ph.hpp"
#include "esp_log.h"

static const char* TAG = "ph";

// 2-point calibration constants
static const float PH_CAL_M = 7.425742574257425f;   // slope (pH per V) 2 point calibration
static const float PH_CAL_B = 0.04950495049504955f; // offset

// Temperature compensation
#define TEMP_COEFF -0.03f // pH units per °C deviation from 25°C
#define REF_TEMP 25.0f    // Reference temperature (°C)

PH::PH(Adc& adc, const AdcConfig& config, size_t channel_idx, const std::vector<SensorData>& sensor_data)
    : Sensor(SensorData::Type::PH), 
    adc_(adc), config_(config), 
    channel_idx_(channel_idx), 
    sensor_data_(sensor_data) {
    ESP_LOGI(TAG, "PH Sensor initialized on ADC channel %d with 12dB attenuation", config_.channel);
}

esp_err_t PH::read(float& value) {
    float voltage;
    esp_err_t ret = adc_.read(channel_idx_, voltage);

    if (ret == ESP_OK) {
        if (voltage < 0.0f || voltage > 3.3f) {
            ESP_LOGW(TAG, "Voltage out of range: %.3fV", voltage);
            value = 0.0f;
            return ESP_ERR_INVALID_RESPONSE;
        }

        // linear calibration formula
        float pH_uncompensated = PH_CAL_M * voltage + PH_CAL_B;

        // Temperature compensation
        float temperature = 0.0f;
        for (const auto& data : sensor_data_) {
            if (data.type == SensorData::Type::NTC) {
                temperature = data.value;
                break;
            }
        }
        float pH_compensated = pH_uncompensated + (TEMP_COEFF * (temperature - REF_TEMP));

        // Range clamp
        value = (pH_compensated < 0.0f) ? 0.0f : (pH_compensated > 14.0f) ? 14.0f : pH_compensated;

        ESP_LOGD(TAG, "PH Sensor: Voltage=%.3fV, Temp=%.2f°C, pH_uncomp=%.3f, pH_comp=%.3f",
                 voltage, temperature, pH_uncompensated, value);
    } else {
        ESP_LOGW(TAG, "PH Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }
    return ret;
}