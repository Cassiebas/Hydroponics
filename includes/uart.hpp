#ifndef UART_HPP
#define UART_HPP

#include <cstdint>
#include <vector>
#include "driver/uart.h"
#include "driver/gpio.h"

constexpr size_t RX_BUFFER_SIZE = 128;
constexpr int READ_TIMEOUT_MS = 100;

struct UartConfig {
    uart_port_t port;       // UART port (e.g., UART_NUM_2)
    int tx_pin;             // TX GPIO
    int rx_pin;             // RX GPIO
    int baud_rate;          // Baud rate (e.g., 9600)
};

class Uart {
    UartConfig config_;
    uint8_t* rx_buffer_;
    void init();

    public:
        Uart(const UartConfig& config);
        ~Uart();
        bool read_sen0311_distance(float& distance_cm);
};

#endif // UART_HPP