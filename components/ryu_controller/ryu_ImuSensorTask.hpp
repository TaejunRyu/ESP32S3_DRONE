/**
 * @file ryu_ImuSensorTask.hpp
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ryu_Types.hpp"


namespace Controller { 

class ImuSensorTask {
        ImuSensorTask() = default; 
        ~ImuSensorTask() = default;
        static constexpr const char* TAG = "ImuSensorTask";
    public:
        static ImuSensorTask& getInstance() {
            static ImuSensorTask instance; 
            return instance;
        }
        ImuSensorTask(const ImuSensorTask&) = delete;
        ImuSensorTask& operator=(const ImuSensorTask&) = delete;
        ImuSensorTask(ImuSensorTask&&) = delete;
        ImuSensorTask& operator=(ImuSensorTask&&) = delete;

        // icm20948 cs pin
        inline static constexpr int SPI_CS_PIN = 9;
     
        // [방어 설계] 생성자에서 모든 포인터를 nullptr로 확실하게 초기화하여 쓰레기 값을 방지합니다.
        bool is_initialized(){return _initialized;};
        esp_err_t initialize();
         
        static void readImuSensorTask(void* pvParameters);
        void StartTask();
        TaskHandle_t getTaskHandle(){return _taskHandle;};
    private:
        TaskHandle_t _taskHandle = nullptr;
        bool _initialized = false;
    };

} // namespace Service
