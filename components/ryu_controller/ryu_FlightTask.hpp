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

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
        
    private:
        TaskHandle_t _taskHandle = nullptr;
        bool _initialized = false;
};



}// namespace Controller