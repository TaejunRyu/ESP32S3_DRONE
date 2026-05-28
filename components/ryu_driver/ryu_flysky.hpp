#pragma once

#include <cstdint>
#include <cstddef>
#include <esp_err.h>

#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace Driver {


class Flysky {
    
private:
    static constexpr const char* TAG = "FLYSKY";

public:
    Flysky(uart_port_t port = UART_NUM_1);
    ~Flysky();

    esp_err_t initialize();
    void deinitialize();

    // RC 채널 데이터 읽기
    uint16_t get_channel(uint8_t channel) const;
    const uint16_t* get_all_channels() const;

    // 상태 확인
    bool is_connected() const;

    // UART 설정
    void set_uart_baud(int baud) { uart_baud_ = baud; }
    void set_uart_pins(gpio_num_t tx, gpio_num_t rx);

private:
    
    static const constexpr gpio_num_t TX_PIN = GPIO_NUM_17;
    static const constexpr gpio_num_t RX_PIN = GPIO_NUM_18;
    static const constexpr size_t IBUS_CHANNELS = 14;
    static const constexpr size_t IBUS_PACKET_SIZE = 32;
    
    uart_port_t port_   = UART_NUM_1;
    int uart_baud_  = 115200;
    gpio_num_t tx_pin_ = TX_PIN;
    gpio_num_t rx_pin_ = RX_PIN;
    bool initialized_ = false;
    bool is_connected_ = false ;
    TaskHandle_t rx_task_handle_ = nullptr;
    QueueHandle_t data_queue_ = nullptr;
    uint16_t rc_values_[IBUS_CHANNELS] = {0};

    // 데이터 처리
    esp_err_t parse_ibus_packet(const uint8_t* packet, size_t len);

    // 수신 태스크
    static void flysky_rx_task(void* arg);

    static const int UART_BUF_SIZE = 256;
    static const int BAUD_RATE = 115200;
};
 
} // namespace Driver
