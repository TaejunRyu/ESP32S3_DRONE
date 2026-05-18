#pragma once

#include <driver/i2c_master.h>
#include <driver/gpio.h>

namespace Driver{

class I2C{
 private:
        I2C() = default; 
        ~I2C() = default;
        static constexpr const char* TAG = "I2C";
    public:
        static I2C& get_instance() {
            static I2C instance; 
            return instance;
        }
        I2C(const I2C&) = delete;
        I2C& operator=(const I2C&) = delete;
        I2C(I2C&&) = delete;
        I2C& operator=(I2C&&) = delete;


        static inline constexpr uint32_t       I2C_SPEED   = 400'000;
        static inline constexpr gpio_port_t    I2C_PORT    = gpio_port_t(0);
        static inline constexpr gpio_num_t     I2C_SDA     = GPIO_NUM_8; // 숫자에 직접 타입을 지정
        static inline constexpr gpio_num_t     I2C_SCL     = GPIO_NUM_9;


        i2c_master_bus_handle_t get_bus_handle(){return _bus_handle;};
        esp_err_t initialize();
        bool is_initialized(){return _initialized;};
        esp_err_t  deinitialize();
        void scan_bus();

    private:
        i2c_port_num_t _port = I2C_PORT;
        gpio_num_t _port_sda = I2C_SDA;
        gpio_num_t _port_scl = I2C_SCL;

        i2c_master_bus_handle_t _bus_handle = nullptr;        
        bool _initialized = false;
};


}