#ifndef NTC_HPP
#define NTC_HPP

#include "sensor.hpp"
#include "adc.hpp"

class NTC : public Sensor {
    Adc& adc_;
    AdcConfig config_;
    size_t channel_idx_;
    
    public:
        NTC(Adc& adc, const AdcConfig& config, size_t channel_idx);
        esp_err_t read(float& value) override;
};

#endif // NTC_SENSOR_HPP