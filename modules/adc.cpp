#include <stdio.h>
#include <string.h>
#include <math.h>

#include "adc.hpp"
#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"

static const char* TAG = "adc";

Adc::Adc(const std::vector<AdcConfig_t>& configs)
    : handle_(nullptr),
      configs_(configs),
      cali_handles_(configs.size(), nullptr),
      do_calibration_(configs.size(), false),
      voltage_history_(configs.size(), std::vector<float>(MOVING_AVG_WINDOW, 0.0f)),
      voltage_history_index_(configs.size(), 0),
      reference_voltages_(configs.size()) {
    if (configs.empty()) {
        ESP_LOGE(TAG, "No ADC configurations provided");
        return;
    }
    for (size_t i = 0; i < configs.size(); ++i) {
        reference_voltages_[i] = configs[i].atten == ADC_ATTEN_DB_6 ? 2.15f : 3.3f;
    }
    init();
}

Adc::~Adc() {
    if (handle_) {
        adc_oneshot_del_unit(handle_);
    }
    for (size_t i = 0; i < cali_handles_.size(); ++i) {
        if (do_calibration_[i]) {
            ESP_LOGI(TAG, "Deregister calibration for channel %d", configs_[i].channel);
            #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
                adc_cali_delete_scheme_curve_fitting(cali_handles_[i]);
            #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
                adc_cali_delete_scheme_line_fitting(cali_handles_[i]);
            #endif
        }
    }
}

void Adc::init() {
    if (configs_.empty()) {
        ESP_LOGE(TAG, "Cannot initialize ADC: No channels configured");
        return;
    }

    // Initialize ADC unit
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &handle_));

    // Configure channels and initialize calibration
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (size_t i = 0; i < configs_.size(); ++i) {
        chan_cfg.atten = configs_[i].atten;
        ESP_ERROR_CHECK(adc_oneshot_config_channel(handle_, configs_[i].channel, &chan_cfg));

        // Initialize calibration
        #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = configs_[i].channel,
            .atten = configs_[i].atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handles_[i]) == ESP_OK) {
            do_calibration_[i] = true;
            ESP_LOGI(TAG, "Calibration (Curve Fitting) enabled for channel %d", configs_[i].channel);
        }
        #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = configs_[i].atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_config, &cali_handles_[i]) == ESP_OK) {
            do_calibration_[i] = true;
            ESP_LOGI(TAG, "Calibration (Line Fitting) enabled for channel %d", configs_[i].channel);
        }
        #endif
        if (!do_calibration_[i]) {
            ESP_LOGW(TAG, "Calibration not supported for channel %d, using raw ADC", configs_[i].channel);
        }
    }

    ESP_LOGI(TAG, "ADC initialized for %d channels", configs_.size());
}

float Adc::compute_moving_average(float new_voltage, size_t channel_idx) {
    voltage_history_[channel_idx][voltage_history_index_[channel_idx]] = new_voltage;
    voltage_history_index_[channel_idx] = (voltage_history_index_[channel_idx] + 1) % MOVING_AVG_WINDOW;
    float sum = 0;
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        sum += voltage_history_[channel_idx][i];
    }
    return sum / MOVING_AVG_WINDOW;
}

esp_err_t Adc::read(size_t channel_idx, float& voltage, float& value) {
    if (channel_idx >= configs_.size()) {
        ESP_LOGE(TAG, "Invalid channel index: %d", channel_idx);
        return ESP_ERR_INVALID_ARG;
    }

    // Read raw ADC value
    int raw_adc = 0;
    esp_err_t ret = adc_oneshot_read(handle_, configs_[channel_idx].channel, &raw_adc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read error for channel %d: %s", configs_[channel_idx].channel, esp_err_to_name(ret));
        return ret;
    }

    // Convert to voltage
    if (do_calibration_[channel_idx]) {
        int cali_voltage_mv = 0;
        ret = adc_cali_raw_to_voltage(cali_handles_[channel_idx], raw_adc, &cali_voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration error for channel %d: %s", configs_[channel_idx].channel, esp_err_to_name(ret));
            return ret;
        }
        voltage = cali_voltage_mv / 1000.0f; // Convert mV to V
    } else {
        voltage = raw_adc * reference_voltages_[channel_idx] / 4095.0f; // Fallback to raw conversion
    }

    if (voltage < 0) voltage = 0;
    if (voltage > reference_voltages_[channel_idx] * 0.99f) {
        ESP_LOGW(TAG, "Warning: Channel %d voltage %.3fV exceeds safe range!", channel_idx, voltage);
        return ESP_ERR_INVALID_STATE;
    }

    float smoothed_voltage = compute_moving_average(voltage, channel_idx);

    // Sensor-specific calculations
    if (configs_[channel_idx].channel == ADC_CHANNEL_1) {
        value = smoothed_voltage * 1667.0f; // TDS calculation
        // ESP_LOGI(TAG, "Channel %d (TDS): ADC: %d, Voltage: %.3fV, Smoothed: %.3fV, TDS: %.0f ppm",
        //          channel_idx, raw_adc, voltage, smoothed_voltage, value);
    } else if (configs_[channel_idx].channel == ADC_CHANNEL_2) {
        const float R_REF = 10000.0f;
        if (smoothed_voltage < 0.01f || smoothed_voltage > 3.29f) {
            ESP_LOGW(TAG, "Invalid NTC voltage: %.3fV", smoothed_voltage);
            value = 0.0f;
            return ESP_ERR_INVALID_STATE;
        }
        float r_ntc = R_REF * ((3.3f / smoothed_voltage) - 1.0f);
        const float A = 0.8999648402e-3f;
        const float B = 2.494581846e-4f;
        const float C = 2.002476456e-7f;
        float ln_r = log(r_ntc);
        float ln_r_cube = ln_r * ln_r * ln_r;
        float temp_k = 1.0f / (A + B * ln_r + C * ln_r_cube);
        float temp_c = temp_k - 273.15f;
        value = temp_c;
        // ESP_LOGI(TAG, "Channel %d (NTC): ADC: %d, Voltage: %.3fV, Smoothed: %.3fV, Resistance: %.0fΩ, Temperature: %.2f°C",
        //          channel_idx, raw_adc, voltage, smoothed_voltage, r_ntc, temp_c);
        // ESP_LOGD(TAG, "NTC: ln(R)=%.3f, ln(R)^3=%.3f, Temp_K=%.2f", ln_r, ln_r_cube, temp_k);
    } else {
        value = smoothed_voltage * 1000.0f;
        // ESP_LOGI(TAG, "Channel %d: ADC: %d, Voltage: %.3fV, Smoothed: %.3fV, Value: %.2f",
        //          channel_idx, raw_adc, voltage, smoothed_voltage, value);
    }

    return ESP_OK;
}