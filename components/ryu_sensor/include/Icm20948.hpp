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
        esp_err_t   initialize() override;
        esp_err_t   deinitialize()  override;
        bool        is_initialized() { return _initialized; }
        esp_err_t   updateSample(ImuData& sample) override;
        esp_err_t   do_calibration();
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

        // AK09916
        static inline constexpr float MAG_SCALE_X       =   0.91f;
        static inline constexpr float MAG_SCALE_Y       =   1.07f;
        static inline constexpr float MAG_SCALE_Z       =   1.04f;
        static inline constexpr float MAG_OFFSET_X      =   91.5f;
        static inline constexpr float MAG_OFFSET_Y      =   176.0f;
        static inline constexpr float MAG_OFFSET_Z      =   -170.5f;

        // 센서 원시 데이터(LSB)를 (g)(중력가속도 단위, 약 9.81 m/s^2)로 변환하는 스케일 팩터입니다.
        // 16bit ADC 절반 크기인 32768/8g = 4096LSB/g
        static inline constexpr float ACCEL_SENSITIVITY    = 4096.0f;   // -+8g Full-Scale Range
        //센서 원시 데이터(LSB)를 deg/s (DPS, 초당 회전 각도)로 변환하는 스케일 팩터입니다.
        // 16bit ADC 절반 크기인 32768/(1000 deg/s) = 32.768LSB/g
        static inline constexpr float GYRO_SENSITIVITY     = 32.768f;     // -+1000도 Full-Scale Range
        // MAG 원시데이터 (LSB) 에 0.15를 곱해줘야 마이크로테슬라가 된다.
        static inline constexpr float MAG_SENSITIVITY      = 0.15f;     
        esp_err_t enable_mag_bypass();
        esp_err_t select_bank(uint8_t bank);
        esp_err_t read_data(ImuData& raw);

        const Vector3f _mag_scale   {MAG_SCALE_X,MAG_SCALE_Y,MAG_SCALE_Z};
        const Vector3f _mag_offset  {MAG_OFFSET_X,MAG_OFFSET_Y,MAG_OFFSET_Z};
        Vector3f _acc_offset  {0.0f,0.0f,0.0f};
        Vector3f _gyro_offset {0.0f,0.0f,0.0f};
        bool _calibration = false;
        bool _initialized = false;
        Interface::IBus* _ibus = nullptr;
};


}// namespace Sensor