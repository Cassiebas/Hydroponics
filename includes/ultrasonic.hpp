#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include "sensor.hpp"
#include "uart.hpp"

class Ultrasonic : public Sensor {
    Uart& uart_;
    public:
        Ultrasonic(Uart& uart);
        esp_err_t read(float& value) override;
};

#endif // ULTRASONIC_SENSOR_HPP