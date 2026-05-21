#pragma once

#include <cstdint>
#include <esp_err.h>
#include <led_strip.h>
#include <freertos/FreeRTOS.h>

namespace Drivers {

class LedStripDriver {
private:
    static constexpr const char* TAG = "LedStripDriver";
    // 싱글톤 기본 생성자는 private 유지
    LedStripDriver() : gpio_num_(48), max_leds_(1) {}
    enum class LedMode { OFF, OK, ERROR, BUSY, WARNING };
    LedMode current_mode_ = LedMode::OFF;

public:
    // 소멸자는 public에 위치해야 static 객체가 정상 소멸됩니다.
    ~LedStripDriver() = default;

    static LedStripDriver& getInstance() { 
        static LedStripDriver instance;
        return instance;
    }

    // 싱글톤 패턴: 복사 및 이동 방지
    LedStripDriver(const LedStripDriver&) = delete;
    LedStripDriver& operator=(const LedStripDriver&) = delete;
    LedStripDriver(LedStripDriver&&) = delete;
    LedStripDriver& operator=(LedStripDriver&&) = delete;

    esp_err_t initialize();
    void deinitialize();

    // 색상 제어
    void set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
    void update();  // 변경사항 적용
    void clear();   // 모든 LED 끄기

    // 상태 표시
    void set_status_ok();      // 녹색 - 정상
    void set_status_error();   // 빨간색 - 에러
    void set_status_busy();    // 파란색 - 처리 중
    void set_status_warning(); // 노란색 - 경고
    void init_sequence();
    
    // 백그라운드 효과를 실행할 static 함수
    static void led_effect_task(void* pvParameters);
    void stop_current_effect();
    void start_effect(LedMode mode);

private:
    TaskHandle_t blink_task_handle_ = nullptr;

    led_strip_handle_t strip_handle_ = nullptr;
    uint8_t gpio_num_ = 48;
    uint8_t max_leds_ = 1;
    bool initialized_ = false;
};

} // namespace Drivers
