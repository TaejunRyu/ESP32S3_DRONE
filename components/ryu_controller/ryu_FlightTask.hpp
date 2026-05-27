/**
 * @file ryu_FlightTask.hpp
 * @author your name (you@domain.com)
 * @brief 
 *      1. kalman filter && pid controller를 처리하는 태스크 
 * @version 0.1
 * @date 2026-05-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include <algorithm>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_Types.hpp"
namespace Controller{

class Flight{    
    private: 
        static constexpr const char* TAG = "Flight";

    public:
        static Flight& getInstance() {
            static Flight instance;
            return instance;
        }
        Flight() = default;
        ~Flight() = default;
        Flight(const Flight&) = delete;
        Flight& operator=(const Flight&) = delete;

        esp_err_t initialize();
        esp_err_t deinitialize();
        static void flight_task(void* pvParameters);
        TaskHandle_t getTaskHandle(){return _taskHandle;};        

        esp_err_t StartTask();
        // 2d rotation 조립하고 지자계의 정렬을 위한 함수.
        Vector3f rotateMagVector(const Vector3f &raw_mag, float rot_rad);
        uint32_t throttle_to_pwm(float throttle){
            const float clamped_pct = std::clamp(throttle, 0.0f, 100.0f);
            return static_cast<uint32_t>(1000.0f + (clamped_pct / 100.0f) * 1000.0f);

        };
    private:
        TaskHandle_t _taskHandle = nullptr;
        bool _initialized = false;
};



}// namespace Controller