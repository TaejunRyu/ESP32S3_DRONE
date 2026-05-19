#include "ryu_timer.hpp"

#include <esp_log.h>

namespace Service {


esp_err_t Timer::intiallize()
{
    if (_initialized) return ESP_OK;
    // 100ms 단위로 처리됨.
    _timer_handle = xTimerCreate(
                "xTimer_xMs", 
                pdMS_TO_TICKS(100), 
                pdTRUE,
                this, 
                timer_callback);
    
    if (_timer_handle == nullptr) {
        ESP_LOGE(TAG, "Timer creation failed");
        return ESP_ERR_NO_MEM;
    }

    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

esp_err_t Timer::deintiallize()
{
    Stop();
    if (_timer_handle) {
        xTimerDelete(_timer_handle, 0);
        _timer_handle = nullptr;
    }
    ESP_LOGI(TAG, "TimerService destroyed");
    return ESP_OK;
}

esp_err_t Timer::Start()
{
 if (!_initialized || _running) return ESP_OK;

    if (xTimerStart(_timer_handle, 0) != pdPASS) {
        ESP_LOGE(TAG, "Timer start failed");
        return ESP_FAIL;
    }

    _running = true;
    ESP_LOGI(TAG, "TimerService started");
    return ESP_OK;

}

void Timer::Stop()
{
    if (_running && _timer_handle) {
        xTimerStop(_timer_handle, 0);
        _running = false;
        ESP_LOGI(TAG, "TimerService stopped");
    }

}

void Timer::Restart()
{
    // 타이머 재시작
    xTimerReset(_timer_handle, 0);
}

void Timer::Delete()
{
    xTimerDelete(_timer_handle, 0);
}

void Timer::Change_Period(uint16_t n)
{
    // 주기를 n ms로 변경
    xTimerChangePeriod(_timer_handle, pdMS_TO_TICKS(n), 0);

}

//  이디에 있는 함수든 _timer_callback에 저장해서 timer를 실행하도록 해주는 방법
void Timer::set_timer_callback(std::function<void()> callback) {
    _timer_callback = callback;
}

// timer callback에 설정한다. (그러면 위 함수에서 정의된  callback함수를 실행하게 된다.)
void Timer::timer_callback(TimerHandle_t xTimer) {
    Timer* service = static_cast<Timer*>(pvTimerGetTimerID(xTimer));

    if (service && service->_timer_callback) {
        service->_timer_callback();
    }
}

} // namespace TIMER