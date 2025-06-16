#include "ph.hpp"
#include "esp_log.h"

static const char* TAG = "ph";

// Shunt resistor value (Ohms) to convert 4-20mA to voltage (0.6V-3.0V with 150Ω)
#define SHUNT_RESISTOR 150.0f

// Current range for S-pH-01 (mA)
#define MIN_CURRENT 4.0f
#define MAX_CURRENT 20.0f

PH::PH(Adc& adc, const AdcConfig& config, size_t channel_idx)
    : Sensor(SensorData::Type::PH), adc_(adc), config_(config), channel_idx_(channel_idx) {
    ESP_LOGI(TAG, "PH Sensor initialized on ADC channel %d", config_.channel);
}

esp_err_t PH::read(float& value) { 
    float voltage;
    esp_err_t ret = adc_.read(channel_idx_, voltage, value); if (ret == ESP_OK) {
    //Convert voltage to current (I = V / R)
    float current_mA = (voltage / SHUNT_RESISTOR) * 1000.0f;

    // Validate current range (4-20mA)
    if (current_mA < MIN_CURRENT - 0.5f || current_mA > MAX_CURRENT + 0.5f) {
        ESP_LOGW(TAG, "Current out of range: %.2f mA (expected 4-20 mA)", current_mA);
        value = 0.0f;
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Calculate pH using S-pH-01 formula: pH = 14 * (current - 4) / 16
    value = 14.0f * (current_mA - MIN_CURRENT) / 16.0f;
    
    // Ensure pH is within 0-14 range
    if (value < 0.0f){
        value = 0.0f;
    }
    else if (value > 14.0f) {
        value = 14.0f;
    }

    ESP_LOGD(TAG, "pH Sensor: Voltage=%.3fV, Current=%.2f mA, pH=%.2f", voltage, current_mA, value);
    } else {
        ESP_LOGW(TAG, "pH Sensor read failed: %s", esp_err_to_name(ret));
        value = 0.0f;
    }
    return ret;
}