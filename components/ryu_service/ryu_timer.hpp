#pragma once

#include <functional>
#include <freertos/FreeRTOS.h>  // FreeRTOS 기본 설정 및 정의
#include <freertos/timers.h>    // 소프트웨어 타이머 API 전용 헤더

namespace Service{ 

class Timer{
 private:
        Timer() = default; 
        ~Timer() = default;
        static constexpr const char* TAG = "Timer";
    public:
        static Timer& get_instance() {
            static Timer instance; 
            return instance;
        }
        Timer(const Timer&) = delete;
        Timer& operator=(const Timer&) = delete;
        Timer(Timer&&) = delete;
        Timer& operator=(Timer&&) = delete;

        esp_err_t intiallize();
        bool is_initialized(){return _initialized;};
        esp_err_t deintiallize();
        
        // 콜백 설정
        void set_timer_callback(std::function<void()> callback);

        // 다른 class의 method을 연결해서 실행할때 사용.
        static void timer_callback(TimerHandle_t xTimer);

        esp_err_t Start();
        bool is_running(){return _running;};
        void Stop();
        void Restart();
        void Delete();
        void Change_Period(uint16_t n);
    private:
        std::function<void()> _timer_callback;
        
        TimerHandle_t _timer_handle = nullptr;
        bool _initialized = false;
        bool _running = false;

};




} //namespace Service