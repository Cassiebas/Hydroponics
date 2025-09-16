#ifndef PH_HPP
#define PH_HPP

#include "sensor.hpp"
#include "adc.hpp"

class PH : public Sensor {
    Adc& adc_;
    AdcConfig config_;
    size_t channel_idx_;
    const std::vector<SensorData>& sensor_data_;  // Reference to StateMachine's sensor_data_
    
    public:
        PH(Adc& adc, const AdcConfig& config, size_t channel_idx, const std::vector<SensorData>& sensor_data);
        esp_err_t read(float& value) override;
};

#endif // PH_HPP