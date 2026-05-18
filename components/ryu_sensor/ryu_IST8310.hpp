#pragma once

#include <array> 
#include <tuple>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include "ryu_BusInterface.hpp"
#include "ryu_ImuInterface.hpp"

namespace Interface {
    class IBus;
}

namespace Sensor
{

class IST8310  : public Interface::IImuSensor{
    private:
        static constexpr const char* TAG = "IST8310";
    public:
        IST8310();
        ~IST8310();
        static inline constexpr uint8_t ADDR       =    0x0E; // 기본 주소 (ADR핀 상태에 따라 다를 수 있음)

        // 인터페이스 주입 (핵심!)
        void set_bus(Interface::IBus* bus) { _ibus = bus; }
        Interface::IBus* get_bus(){ return _ibus;};    
        esp_err_t initialize() override;
        esp_err_t deinitialize() override;
        esp_err_t updateSample(ImuData &sample) override;
        bool is_initialized(){return _initialized;};
        
        esp_err_t read_data(Vector3f data);
        esp_err_t read_with_offset(Vector3f data);
        void calibrate_hard_iron();

    private:
        static inline constexpr float FILTER_ALPHA = 0.4f; 

        // IST8310 레지스터 정의
        static inline constexpr uint8_t WHO_AM_I    =    0x00; // ID 확인용 (값: 0x10)
        static inline constexpr uint8_t STAT1       =    0x02; // 데이터 준비 상태 (Bit 0: DRDY)
        static inline constexpr uint8_t DATA_X_L    =    0x03; // 데이터 시작 (X-axis Low)
        static inline constexpr uint8_t CONTROL1    =    0x0A; // 모드 설정 (Single/Continuous)
        static inline constexpr uint8_t CONTROL2    =    0x0B; // 소프트 리셋 및 옵션
        static inline constexpr uint8_t AVGCNTL     =    0x41; // 평균 필터 설정 (노이즈 감소)
        static inline constexpr uint8_t PDCNTL      =    0x42; // Pulse Duration 제어
        static inline constexpr uint8_t CROSSAXIS1  =    0x48; // 
        static inline constexpr uint8_t CROSSAXIS2  =    0x49; // 
        // 센서 감도: 1320 LSB/Gauss (0.3 µT/LSB)

        static inline constexpr float SENSITIVITY   =  0.3f;   
        
        static inline constexpr float MAG_OFFSET_X  =  11.85f;
        static inline constexpr float MAG_OFFSET_Y  =  4.35f;
        static inline constexpr float MAG_OFFSET_Z  =  -15.30f;
        static inline constexpr float MAG_SCALE_X   =  0.98f;
        static inline constexpr float MAG_SCALE_Y   =  0.99f;
        static inline constexpr float MAG_SCALE_Z   =  1.03f;
        
        Interface::IBus* _ibus = nullptr; // 하드웨어 추상화 레이어
        Vector3f _mag_offset ={MAG_OFFSET_X,MAG_OFFSET_Y,MAG_OFFSET_Z};
        Vector3f _mag_scale ={MAG_SCALE_X,MAG_SCALE_Y,MAG_SCALE_Z};
        Vector3f _mag_previous {};
        bool _initialized = false;
};





}