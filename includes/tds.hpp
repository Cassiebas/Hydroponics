#ifndef TDS_HPP
#define TDS_HPP

#include "sensor.hpp"
#include "adc.hpp"

class TDS : public Sensor {
    Adc& adc_;
    AdcConfig config_;
    size_t channel_idx_;
    
    public:
        TDS(Adc& adc, const AdcConfig& config, size_t channel_idx);
        esp_err_t read(float& value) override;

};

#endif // TDS_SENSOR_HPP