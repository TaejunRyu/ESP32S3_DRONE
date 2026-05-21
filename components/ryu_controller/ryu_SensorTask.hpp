/**
 * @file ryu_SensorTask.hpp
 * @author your name (you@domain.com)
 * @brief 
 *      1. 데이터가 이동이 잦기 때문에 Controller에 있게함.
 *      2. CORE0 에서 IMU DATA를 Flight task에 공급한다.
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <esp_err.h>
#include "ryu_Types.hpp"

// 전방 선언 (Forward Declaration)을 통해 컴파일 속도를 최적화합니다.
namespace Sensor {
    class ICM20948;  // SPI BUS 사용
    class IST8310;   // I2C BUS 사용   
    class BMP388;    // SPI BUS 사용   ==>  이곳이 아닌 다른 task에서 늦게 돌려서 처리해야 할것 같음....
}

namespace Controller { 

class SensorTask {
    private:        
        static constexpr const char* TAG = "SensorTask";
    public:
        SensorTask() = default;
        ~SensorTask() = default;
        inline static constexpr int SPI_IMU_CS_PIN = 9;
        inline static constexpr int SPI_BMP_CS_PIN = 10;
        // [방어 설계] 생성자에서 모든 포인터를 nullptr로 확실하게 초기화하여 쓰레기 값을 방지합니다.
        bool is_initialized(){return _initialized;};
        esp_err_t initialize();

        static void ReadSensorTask(void* pvParameters);
        void StartTask();
    private:
        bool _initialized = false;
    };

} // namespace Service
