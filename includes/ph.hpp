#ifndef PH_HPP
#define PH_HPP

#include "sensor.hpp"
#include "adc.hpp"

class PH : public Sensor {
    Adc& adc_;
    AdcConfig config_;
    size_t channel_idx_;
    
    public:
        PH(Adc& adc, const AdcConfig& config, size_t channel_idx);
        esp_err_t read(float& value) override;

};

#endif // PH_HPP