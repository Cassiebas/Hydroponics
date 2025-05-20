#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "adc.hpp"
#include "uart.hpp"
#include "sensor.hpp"
#include <vector>
#include <memory>

class StateMachine {
public:
    enum class State {
        SENSOR_DATA_ACQUISITION,
        ACTUATOR_CONTROL,
        MQTT_COMMUNICATION
    };

    enum class ActuatorSubstate {
        ON_OFF_CONTROL,
        PID_CONTROL,
        FUZZY_LOGIC_CONTROL
    };

    StateMachine(const std::vector<AdcConfig>& adc_configs, const UartConfig& uart_config, float tank_height_cm);
    void run();

private:
    Adc adc_;
    Uart uart_;
    float tank_height_cm_;
    std::vector<std::unique_ptr<Sensor>> sensors_;
    std::vector<SensorData> sensor_data_;
    State current_state_;
    ActuatorSubstate actuator_substate_;
    // Placeholder thresholds (could be updated via MQTT)
    float tds_threshold_ = 1000.0f; // ppm
    float temp_threshold_ = 25.0f;  // °C
    float water_level_threshold_ = 50.0f; // cm

    void sensor_data_acquisition();
    void actuator_control();
    void mqtt_communication();
    void on_off_control();
    void pid_control();
    void fuzzy_logic_control();
};

#endif // STATE_MACHINE_HPP