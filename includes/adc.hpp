#ifndef ADC_HPP
#define ADC_HPP

#include <vector>
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

typedef struct AdcConfig {
    adc_channel_t channel;
    adc_atten_t atten;
} AdcConfig_t;

static constexpr size_t MOVING_AVG_WINDOW = 16;

class Adc {
    adc_oneshot_unit_handle_t handle_;
    std::vector<AdcConfig_t> configs_;
    std::vector<adc_cali_handle_t> cali_handles_;
    std::vector<bool> do_calibration_;
    std::vector<std::vector<float>> voltage_history_;
    std::vector<size_t> voltage_history_index_;
    std::vector<float> reference_voltages_;

    public:
        Adc(const std::vector<AdcConfig_t>& configs);
        ~Adc();
        esp_err_t read(size_t channel_idx, float& voltage, float& value);

    private:
        void init();
        float compute_moving_average(float new_voltage, size_t channel_idx);
};

#endif