#pragma once
#include <memory>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "ryu_ImuInterface.hpp"
#include "ryu_FrameTransformer.hpp"

namespace Interface{
    class IBus;
}

namespace Sensor{

class ICM20948 : public Interface::IImuSensor{
    private:
        static constexpr const char* TAG = "ICM20948";
    public:
        ICM20948();
        ~ICM20948();
        static inline constexpr uint8_t ADDR_VCC = 0x69;
        static inline constexpr uint8_t ADDR_GND = 0x68;
        
        // 인터페이스 주입 (핵심!)
        esp_err_t init_bus(Interface::IBus *bus);
        void set_bus(Interface::IBus* bus);
        Interface::IBus* get_bus(){ return _ibus;};   

        // 상태 관리
        esp_err_t   initialize() override;
        esp_err_t   deinitialize()  override;
        esp_err_t   updateSample(ImuData& sample) override;
        esp_err_t   read_data(ImuData& raw);
        esp_err_t   enable_mag_bypass();
        bool        is_initialized() { return _initialized; };
        esp_err_t   calibration_loop(const ImuData &data, int sample_count);
        bool        is_calibration(){return _calibration;};
        // filter 처리용.
        void apply_filter(ImuData& io_data);

        void align_NED(ImuData& data){
            data.current_coordSystem = Utils::CoordSystem::NED;
            data.acc.y  *= -1.0f;  
            data.gyro.x *= -1.0f;
            data.gyro.z *= -1.0f;
            data.mag.x  *= -1.0f;

            //data.mag.z  *= -1.0f;
        }

        void calibration_mag_hard_iron();
        Vector3f get_acc_bias(){return _acc_bias;};
        Vector3f get_gyro_bias(){return _gyro_bias;};
        Vector3f get_mag_offset(){return _mag_offset;};
        Vector3f get_mag_scale(){return _mag_scale;};
        Vector3f get_mag_previous(){return _mag_previous;};
        void set_mag_previous(Vector3f mag){ _mag_previous = mag;};


    private:
        // LOOP내에서 자동으로 처음 COUNT반큼 돌면서 BIAS측정
        static inline constexpr uint16_t CALIBRATION_COUNT = 1000;    

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
        // USER BANK 3 주요 레지스터 맵 정의
        static inline constexpr uint8_t B3_I2C_SLV0_ADDR    =   0x03;
        static inline constexpr uint8_t B3_I2C_SLV0_REG     =   0x04;
        static inline constexpr uint8_t B3_I2C_SLV0_CTRL    =   0x05;
        static inline constexpr uint8_t B3_I2C_SLV4_ADDR    =   0x13; // SLV4 주소
        static inline constexpr uint8_t B3_I2C_SLV4_REG     =   0x14;
        static inline constexpr uint8_t B3_I2C_SLV4_CTRL    =   0x15;
        static inline constexpr uint8_t B3_I2C_SLV4_DO      =   0x16;
        static inline constexpr uint8_t B3_I2C_MST_CTRL     =   0x01;
        static inline constexpr uint8_t B0_EXT_SLV_SENS_DATA_00 = 0x3B; // SPI 모드에서 지자계 데이터가 들어오는 시작점


        static inline constexpr float MAG_SCALE_X        = 1.0145;
        static inline constexpr float MAG_SCALE_Y        = 0.9987;
        static inline constexpr float MAG_SCALE_Z        = 0.9872;
        static inline constexpr float MAG_OFFSET_X   = -2.6250;
        static inline constexpr float MAG_OFFSET_Y   = 15.8250;
        static inline constexpr float MAG_OFFSET_Z   = 30.9750;

        // 센서 원시 데이터(LSB)를 (g)(중력가속도 단위, 약 9.81 m/s^2)로 변환하는 스케일 팩터입니다.
        // 16bit ADC 절반 크기인 32768/8g = 4096LSB/g
        static inline constexpr float ACCEL_SCALE    =  0.000244141f; // 1.0f/4096.0f;   // -+8g Full-Scale Range
        //센서 원시 데이터(LSB)를 deg/s (DPS, 초당 회전 각도)로 변환하는 스케일 팩터입니다.
        // 16bit ADC 절반 크기인 32768/(1000 deg/s) = 32.768LSB/g
        static inline constexpr float GYRO_SCALE     =  0.030517578f;//1.0f/32.768f;     // -+1000도 Full-Scale Range
        // MAG 원시데이터 (LSB) 에 0.15를 곱해줘야 마이크로테슬라가 된다.
        static inline constexpr float MAG_SCALE      =  0.15f;     

        esp_err_t select_bank(uint8_t bank);

        const float _lpf_alpha = 0.2f; 
        Vector3f _last_filtered_accel;
        Vector3f _last_filtered_gyro;

        // HARD_IRON_CALIBRATION에 이해서 계산되어지는 값. 
        const Vector3f _mag_scale   {MAG_SCALE_X,MAG_SCALE_Y,MAG_SCALE_Z};
        const Vector3f _mag_offset  {MAG_OFFSET_X,MAG_OFFSET_Y,MAG_OFFSET_Z};
        
        // calibration으로 값이 설정되어짐.
        Vector3f _acc_bias  {0.0f,0.0f,0.0f};
        Vector3f _gyro_bias {0.0f,0.0f,0.0f};

        // 이전 측정값 (값의 연속성을 위하여 이전값을 저장한다. 리딩 타임이 다르기 때문에....)
        Vector3f _mag_previous {0.0f,0.0f,0.0f};

        bool _calibration = false;
        bool _initialized = false;
        Interface::IBus* _ibus = nullptr;
};


}// namespace Sensor