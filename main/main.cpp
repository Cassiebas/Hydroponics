#include "state_machine.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "sensor_reader";

void state_machine_task(void* pvParameters) {
    std::vector<AdcConfig> adc_configs = {
        { ADC_CHANNEL_1, ADC_ATTEN_DB_6 },
        { ADC_CHANNEL_2, ADC_ATTEN_DB_12 }
    };

    UartConfig uart_config = {
        .port = UART_NUM_1,
        .tx_pin = GPIO_NUM_16,
        .rx_pin = GPIO_NUM_17,
        .baud_rate = 9600
    };

    const float TANK_HEIGHT_CM = 100.0f;
    StateMachine state_machine(adc_configs, uart_config, TANK_HEIGHT_CM);
    state_machine.run();
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting sensor reader with state machine...");
    xTaskCreate(state_machine_task, "state_machine_task", 4096, NULL, 5, NULL);
}