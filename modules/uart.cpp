#include "uart.hpp"
#include "esp_log.h"

static const char* TAG = "uart";

Uart::Uart(const UartConfig& config)
    : config_(config),
      rx_buffer_(new uint8_t[RX_BUFFER_SIZE]) {
    init();
}

Uart::~Uart() {
    uart_driver_delete(config_.port);
    delete[] rx_buffer_;
}

void Uart::init() {
    uart_config_t uart_config = {
        .baud_rate = config_.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(config_.port, RX_BUFFER_SIZE * 2, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(config_.port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config_.port, config_.tx_pin, config_.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush(config_.port));
    ESP_LOGI(TAG, "UART%d initialized: TX=%d, RX=%d, Baud=%d", config_.port, config_.tx_pin, config_.rx_pin, config_.baud_rate);
}

bool Uart::read_sen0311_distance(float& distance_cm) {
    uint8_t data[4] = {0};
    int bytes_read = 0;

    while (bytes_read < 4) {
        int len = uart_read_bytes(config_.port, &data[bytes_read], 1, READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (len <= 0) {
            ESP_LOGW(TAG, "SEN0311 read failed: len=%d, error=%s", len, esp_err_to_name(len < 0 ? len : ESP_ERR_TIMEOUT));
            uart_flush(config_.port);
            return false;
        }
        ESP_LOGD(TAG, "Read byte: 0x%02X at position %d", data[bytes_read], bytes_read);
        bytes_read += len;

        if (bytes_read == 1 && data[0] != 0xFF) {
            ESP_LOGD(TAG, "Invalid start byte: 0x%02X, resetting", data[0]);
            bytes_read = 0;
        } else if (bytes_read == 4) {
            ESP_LOGD(TAG, "Packet: 0x%02X 0x%02X 0x%02X 0x%02X", data[0], data[1], data[2], data[3]);
            uint8_t checksum = (data[0] + data[1] + data[2]) & 0xFF;
            if (checksum != data[3]) {
                ESP_LOGW(TAG, "SEN0311 checksum error: expected 0x%02X, got 0x%02X", checksum, data[3]);
                bytes_read = 0;
                continue;
            }
            uint16_t distance_mm = (data[1] << 8) + data[2];
            if (distance_mm < 30 || distance_mm > 4500) {
                ESP_LOGW(TAG, "SEN0311 distance out of range: %d mm", distance_mm);
                return false;
            }
            distance_cm = distance_mm / 10.0f;
            return true;
        }
    }
    return false;
}