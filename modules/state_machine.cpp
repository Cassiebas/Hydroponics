#include "state_machine.hpp"
#include "tds.hpp"
#include "ntc.hpp"
#include "ph.hpp"
#include "ultrasonic.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h> // Add this for fflush

static const char* TAG = "state_machine";

StateMachine::StateMachine(const std::vector<AdcConfig>& adc_configs, const UartConfig& uart_config, float tank_height_cm)
    : adc_(adc_configs),
      uart_(uart_config),
      tank_height_cm_(tank_height_cm),
      sensor_data_(4),
      current_state_(State::SENSOR_DATA_ACQUISITION),
      actuator_substate_(ActuatorSubstate::ON_OFF_CONTROL) {
    // Initialize sensors
    sensors_.push_back(std::make_unique<TDS>(adc_, adc_configs[0], 0, sensor_data_));
    sensors_.push_back(std::make_unique<NTC>(adc_, adc_configs[1], 1));
    sensors_.push_back(std::make_unique<PH>(adc_, adc_configs[2], 2, sensor_data_));
    sensors_.push_back(std::make_unique<Ultrasonic>(uart_));

    // Initialize sensor data
    sensor_data_[0].type = SensorData::Type::TDS;
    sensor_data_[1].type = SensorData::Type::NTC;
    sensor_data_[2].type = SensorData::Type::WATER_LEVEL;
    sensor_data_[3].type = SensorData::Type::PH;

    ESP_LOGI(TAG, "State machine initialized with %d sensors", sensors_.size());
}

void StateMachine::run() {
    while (1) {
        switch (current_state_) {
            case State::SENSOR_DATA_ACQUISITION:
                sensor_data_acquisition();
                current_state_ = State::SENSOR_DATA_ACQUISITION;
                break;
            case State::ACTUATOR_CONTROL:
                actuator_control();
                current_state_ = State::MQTT_COMMUNICATION;
                break;
            case State::MQTT_COMMUNICATION:
                mqtt_communication();
                current_state_ = State::SENSOR_DATA_ACQUISITION;
                break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms per cycle
    }
}

void StateMachine::sensor_data_acquisition() {
    for (size_t i = 0; i < sensors_.size(); ++i) {
        float value;
        esp_err_t ret = sensors_[i]->read(value);
        if (ret == ESP_OK) {
            sensor_data_[i].value = value;
        } else {
            sensor_data_[i].value = 0.0f;
        }
    }

    ESP_LOGI(TAG, "TDS: %.2f ppm, pH: %.2f pH, Tem: %.2f °C, Water-Level : %.2f",
             sensor_data_[0].value, sensor_data_[2].value, sensor_data_[1].value, sensor_data_[3].value);
}

void StateMachine::actuator_control() {
    switch (actuator_substate_) {
        case ActuatorSubstate::ON_OFF_CONTROL:
            on_off_control();
            break;
        case ActuatorSubstate::PID_CONTROL:
            pid_control();
            break;
        case ActuatorSubstate::FUZZY_LOGIC_CONTROL:
            fuzzy_logic_control();
            break;
    }
}

void StateMachine::on_off_control() {
    // Placeholder: Simple threshold-based pump control
    bool pump_on = sensor_data_[0].value > tds_threshold_ ||
                   sensor_data_[1].value > temp_threshold_ ||
                   sensor_data_[2].value < water_level_threshold_;
    ESP_LOGI(TAG, "On/Off Control: Pump %s", pump_on ? "ON" : "OFF");
    // Add GPIO control for pump here (e.g., gpio_set_level)
}

void StateMachine::pid_control() {
    // Placeholder: PID control for actuators
    ESP_LOGI(TAG, "PID Control: Processing TDS=%.0f, Temp=%.2f, Level=%.1f",
             sensor_data_[0].value, sensor_data_[1].value, sensor_data_[2].value);
    // Implement PID logic here
}

void StateMachine::fuzzy_logic_control() {
    // Placeholder: Fuzzy logic control
    ESP_LOGI(TAG, "Fuzzy Logic Control: Processing TDS=%.0f, Temp=%.2f, Level=%.1f",
             sensor_data_[0].value, sensor_data_[1].value, sensor_data_[2].value);
    // Implement fuzzy logic here
}

void StateMachine::mqtt_communication() {
    // Placeholder: Publish sensor data and receive threshold updates
    ESP_LOGI(TAG, "MQTT: Publishing TDS=%.0f, Temp=%.2f, Level=%.1f",
             sensor_data_[0].value, sensor_data_[1].value, sensor_data_[2].value);
    // Implement MQTT client logic here (e.g., using esp-mqtt)
}