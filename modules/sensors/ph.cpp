#include "ph.hpp"
#include "esp_log.h"

static const char* TAG = "ph";

// Shunt resistor value (Ohms) to convert 4-20mA to voltage (0.6V-3.0V with 150Ω)
#define SHUNT_RESISTOR 150.0f

// Current range for S-pH-01 (mA)
#define MIN_CURRENT 4.0f
#define MAX_CURRENT 20.0f

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
    // ESP_LOGI(TAG, "pH Voltage: %.3fV", voltage);

    if (ret == ESP_OK) {
        // Step 1: Validate voltage range (0-2V, buffer for safety)
        if (voltage < 0.0f || voltage > 2.5f) {
            ESP_LOGW(TAG, "Voltage out of range: %.3fV (expected ~0-2.5V)", voltage);
            value = 0.0f;
            return ESP_ERR_INVALID_RESPONSE;
        }

        // Step 2: Calculate uncompensated pH with slight offset correction
        float pH_uncompensated = 7.5f * voltage - 0.275f;

        // Step 3: Apply temperature compensation
        float temperature = 0.0f;
        for (const auto& data : sensor_data_) {
            if (data.type == SensorData::Type::NTC) {
                temperature = data.value;
                break;
            }
        }
        float pH_compensated = pH_uncompensated + (TEMP_COEFF * (temperature - REF_TEMP));

        // Step 4: Ensure pH is within 0-14 range
        value = (pH_compensated < 0.0f) ? 0.0f : (pH_compensated > 14.0f) ? 14.0f : pH_compensated;

        ESP_LOGD(TAG, "PH Sensor: Voltage=%.3fV, Temp=%.2f°C, pH_uncomp=%.2f, pH_comp=%.2f",
                 voltage, temperature, pH_uncompensated, value);
    } else {
        ESP_LOGW(TAG, "PH Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }
    return ret;
}