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

namespace Control{

class Flight{
    public:
        esp_err_t initialize();
        esp_err_t deinitialize();
        void flight_task();
        void start_task();
    private:


        bool _initialized = false;
};



}// namespace Control