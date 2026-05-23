#pragma once

#include <math.h>
#include <algorithm>
#include <string>
#include <esp_log.h>
#include "ryu_ISensor.hpp"


namespace Interface{
    class IBus;
}

namespace Sensor{

class BMP388{
    private:
        BMP388()= default;
        static constexpr const char* TAG = "BMP388";
    public:
        static BMP388& getInstance() {
            static BMP388 instance;
            return instance;
        }
        // 싱글톤 패턴: 복사 및 이동 방지
        BMP388(const BMP388&) = delete;
        BMP388& operator=(const BMP388&) = delete;
        ~BMP388();

        // 인터페이스 주입 (핵심!)
        void set_bus(Interface::IBus* bus);
        Interface::IBus* get_bus(){ return _ibus;};    

        // I2C 연결시 사용
        static inline constexpr uint8_t ADDR_VCC   =   0x77;
        static inline constexpr uint8_t ADDR_GND   =   0x76;

        
        esp_err_t initialize() ;
        esp_err_t deinitialize();
        esp_err_t get_pressure(float *pressure);        
        bool is_initialized(){return _initialized;};
        bool is_data_ready();

    private:
        Interface::IBus* _ibus = nullptr;

        static inline constexpr uint8_t REG_ID        =   0x00;
        static inline constexpr uint8_t REG_DATA      =   0x04; // Pressure(3), Temp(3) 연속
        static inline constexpr uint8_t REG_PWR_CTRL  =   0x1B;
        static inline constexpr uint8_t REG_CALIB     =   0x31;
        static inline constexpr uint8_t STATUS        =   0x03;

        //보정계수        
        struct {
            double      t_lin;
            uint16_t    t1; 
            uint16_t    t2;  
            int8_t      t3;
            int16_t     p1, p2; 
            int8_t      p3, p4;
            uint16_t    p5, p6;  
            int8_t      p7, p8;
            int16_t     p9; 
            int8_t      p10, p11;
        } _coef;

        // 보정계수와 미리계산 될것(한번만 하면 되는것들)
        float   _p1 = 0.0f, _p2 = 0.0f, _p3 = 0.0f, _p4 = 0.0f, _p5 = 0.0f, _p6 = 0.0f, 
                _p7 = 0.0f, _p8 = 0.0f, _p9 = 0.0f, _p10= 0.0f, _p11= 0.0f;

        // bmp388에서 읽은 raw data.
        uint32_t uncomp_temp  = 0;
        uint32_t uncomp_press = 0;
        
        // 정상으로 읽은 이전값   현재 데이터가 잘못되면 이전값을 내어준다.      
        uint32_t    adc_p_last = 0, 
                    adc_t_last = 0;
                    
        void init_coefficients();
        inline esp_err_t  read_calib();
        inline esp_err_t  read_bmp388(uint32_t *adcp, uint32_t *adct);
       
        bool _initialized = false;
    };

}


