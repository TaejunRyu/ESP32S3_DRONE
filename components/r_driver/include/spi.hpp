#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>

namespace Driver {

class SPI {
private:
    SPI() = default;
    ~SPI() = default;
    static constexpr const char* TAG = "SPI";

public:
    static SPI& get_instance() {
        static SPI instance;
        return instance;
    }

    // 싱글톤 패턴: 복사 및 이동 방지
    SPI(const SPI&) = delete;
    SPI& operator=(const SPI&) = delete;

    // ESP32-S3 기본 SPI 핀 설정 (원하는 핀으로 변경 가능)
    static inline constexpr spi_host_device_t   SPI_HOST = SPI2_HOST; // FSPI
    static inline constexpr gpio_num_t          SPI_SCLK = GPIO_NUM_12;
    static inline constexpr gpio_num_t          SPI_MOSI = GPIO_NUM_11;
    static inline constexpr gpio_num_t          SPI_MISO = GPIO_NUM_13;

    /**
     * @brief SPI 버스 초기화
     */
    esp_err_t initialize();

    /**
     * @brief SPI 버스 해제
     */
    esp_err_t deinitialize();

    spi_host_device_t get_host() { return SPI_HOST; }
    bool is_initialized() { return _initialized; }

private:
    gpio_num_t _port_sclk = SPI_SCLK;
    gpio_num_t _port_mosi = SPI_MOSI;
    gpio_num_t _port_miso = SPI_MISO;
    bool _initialized = false;
};

} // namespace Driver
