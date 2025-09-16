#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "esp_log.h"
#include "esp_err.h"
#include <vector>

struct SensorData {
    enum class Type {
        TDS,
        NTC,
        WATER_LEVEL,
        PH
    };
    Type type;
    float value; // TDS (ppm), Temperature (°C), or Water Level (cm), etc..
};

class Sensor {
    public:
        virtual ~Sensor() = default;
        virtual esp_err_t read(float& value) = 0; // Pure virtual function for reading sensor data
        SensorData::Type get_type() const { return type_; }
    protected:
        Sensor(SensorData::Type type) : type_(type) {}
        SensorData::Type type_;
};

#endif // SENSOR_HPP