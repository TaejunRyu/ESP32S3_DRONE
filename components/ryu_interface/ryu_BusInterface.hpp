#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

namespace Interface{

enum class BusType { I2C, SPI };

// 1. 최상위 인터페이스
class IBus{
public:
    virtual ~IBus() = default;
    virtual BusType get_type() const = 0;
    virtual esp_err_t Write(uint8_t reg, uint8_t data) = 0;
    virtual esp_err_t Read(uint8_t reg, uint8_t* data, size_t len) = 0;
};

// 2. SPI 구현체
class SPIBus : public IBus {
private:
    spi_device_handle_t _handle;
public:
    SPIBus(const SPIBus&) = delete;
    SPIBus& operator=(const SPIBus&) = delete;
    SPIBus(spi_device_handle_t handle) : _handle(handle) {}
    ~SPIBus() override { if (_handle) spi_bus_remove_device(_handle); } // 리소스 해제 추가
    BusType get_type() const override { return BusType::SPI; }
    
    inline esp_err_t Write(uint8_t reg, uint8_t data) override {
        spi_transaction_t t = {};
        t.flags = SPI_TRANS_USE_TXDATA;
        t.addr = reg & 0x7F;
        t.length = 8;
        t.tx_data[0] = data;
        return spi_device_polling_transmit(_handle, &t);
    }

    inline esp_err_t Read(uint8_t reg, uint8_t* data, size_t len) override {
        if (len == 0) return ESP_OK;
        spi_transaction_t t = {};
        t.addr = reg | 0x80;
        t.length = 8 * len;
        t.rxlength = 8 * len;
        t.rx_buffer = data;
        return spi_device_polling_transmit(_handle, &t);
    }
};

// 3. I2C 구현체
class I2CBus : public IBus {
    private:
        i2c_master_dev_handle_t _handle;
    public:
        I2CBus(const I2CBus&) = delete;
        I2CBus& operator=(const I2CBus&) = delete;

        I2CBus(i2c_master_dev_handle_t handle) : _handle(handle) {}
        ~I2CBus() override { if (_handle) i2c_master_bus_rm_device(_handle); }
        BusType get_type() const override { return BusType::I2C; }

        inline esp_err_t Write(uint8_t reg, uint8_t data) override {
            uint8_t buf[2] = {reg, data};
            return i2c_master_transmit(_handle, buf, 2, pdMS_TO_TICKS(2));
        }

        inline esp_err_t Read(uint8_t reg, uint8_t* data, size_t len) override {
            return i2c_master_transmit_receive(_handle, &reg, 1, data, len, pdMS_TO_TICKS(2));
        }
};

inline IBus* createBIF(i2c_master_bus_handle_t bus_handle, uint16_t addr) {
    i2c_master_dev_handle_t dev_h;
    i2c_device_config_t dev_cfg = {}; // 0으로 전체 초기화
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 400000;

    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_h) == ESP_OK) {
        return new I2CBus(dev_h);
    }
    return nullptr;
}

inline IBus* createBIF(spi_host_device_t host, int cs_io) {
    spi_device_handle_t dev_h;
    spi_device_interface_config_t dev_cfg = {}; // 0으로 전체 초기화
    dev_cfg.mode = 3;
    dev_cfg.address_bits = 8;
    dev_cfg.clock_speed_hz = 7 * 1000 * 1000;
    dev_cfg.spics_io_num = cs_io;
    dev_cfg.queue_size = 7;

    if (spi_bus_add_device(host, &dev_cfg, &dev_h) == ESP_OK) {
        return new SPIBus(dev_h);
    }
    return nullptr;
}


} //namespace IBus