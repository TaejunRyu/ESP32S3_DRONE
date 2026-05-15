// components/hardware/include/ICM20948.hpp
#pragma once
#include <memory>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "ImuInterface.hpp"

namespace Interface{
    class IBus;
}

namespace Sensor{

class ICM20948 : public Interface::IImuSensor{
    private:
        static constexpr const char* TAG = "ICM20948";


    public:
        ~ICM20948();
        static inline constexpr uint8_t ADDR_GND = 0x68;
        static inline constexpr uint8_t ADDR_VCC = 0x69;
        

        // 인터페이스 주입 (핵심!)
        esp_err_t init_bus(Interface::IBus *bus);
        void set_bus(Interface::IBus* bus);
        Interface::IBus* get_bus(){ return _ibus;};   

        // 상태 관리
        esp_err_t initialize() override;
        esp_err_t deinitialize()  override;
        bool is_initialized() { return _initialized; }
        esp_err_t updateSample(ImuData& sample) override;

    private:
        
        // I2C && SPI 레지스터 및 상수 정의
        static inline constexpr uint8_t REG_BANK_SEL = 0x7F;
        static inline constexpr uint8_t B0_WHO_AM_I = 0x00;
        static inline constexpr uint8_t B0_PWR_MGMT_1 = 0x06;
        static inline constexpr uint8_t B0_ACCEL_XOUT_H = 0x2D;
        static inline constexpr uint8_t B0_GYRO_XOUT_H = 0x33;
        static inline constexpr uint8_t B0_INT_PIN_CFG = 0x0F;
        static inline constexpr uint8_t B0_USER_CTRL = 0x03;
        static inline constexpr uint8_t B2_GYRO_CONFIG_1 = 0x01;
        static inline constexpr uint8_t B2_ACCEL_CONFIG = 0x14;


        // SPI BYPASS기능을위해... 기존 레지스터 정의 아래에 추가 ...
        static inline constexpr uint8_t B3_I2C_MST_CTRL = 0x01;
        static inline constexpr uint8_t B3_I2C_SLV0_ADDR = 0x03;
        static inline constexpr uint8_t B3_I2C_SLV0_REG = 0x04;
        static inline constexpr uint8_t B3_I2C_SLV0_CTRL = 0x05;
        static inline constexpr uint8_t B0_EXT_SLV_SENS_DATA_00 = 0x3B; // SPI 모드에서 지자계 데이터가 들어오는 시작점


        static constexpr float ACCEL_SENSITIVITY = 4096.0f;
        static constexpr float GYRO_SENSITIVITY  = 32.8f;

        

        esp_err_t select_bank(uint8_t bank);
        esp_err_t read_data(ImuData& raw);

        Vector3f _mag {};
        bool _initialized = false;
        Interface::IBus* _ibus = nullptr;
};


}// namespace Sensor