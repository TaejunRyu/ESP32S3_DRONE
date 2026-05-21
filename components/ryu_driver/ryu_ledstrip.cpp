/**
 * @file ryu_ledstrip.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "ryu_ledstrip.hpp"
#include <cmath>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

namespace Drivers {

esp_err_t LedStripDriver::initialize() {
    if (initialized_) return ESP_OK;

    ESP_LOGI(TAG, "Initializing LED strip on GPIO %u...", gpio_num_);

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = gpio_num_;
    strip_config.max_leds = max_leds_;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = false;

    // RMT 백엔드 설정
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;  // 10MHz

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip initialization failed: %s", esp_err_to_name(ret));
        strip_handle_ = nullptr;
        return ret;
    }

    if (strip_handle_ == nullptr) {
        ESP_LOGE(TAG, "LED strip handle is null");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LedStripDriver created (GPIO: %u, Max LEDs: %u)", gpio_num_, max_leds_);
    initialized_ = true;
    ESP_LOGI(TAG, "LED strip initialized successfully");
    return ESP_OK;
}

void LedStripDriver::deinitialize() {
    if (!initialized_) return;

    if (strip_handle_ != nullptr) {
        led_strip_del(strip_handle_);
        strip_handle_ = nullptr;
    }

    initialized_ = false;
    ESP_LOGI(TAG, "LED strip deinitialized");
}

void LedStripDriver::set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue) {
    if (!initialized_ || strip_handle_ == nullptr) return;

    if (index >= max_leds_) {
        ESP_LOGW(TAG, "Pixel index out of range: %u >= %u", index, max_leds_);
        return;
    }

    led_strip_set_pixel(strip_handle_, index, red, green, blue);
}

void LedStripDriver::update() {
    if (!initialized_ || strip_handle_ == nullptr) return;

    led_strip_refresh(strip_handle_);
}

void LedStripDriver::clear() {
    if (!initialized_ || strip_handle_ == nullptr) return;

    led_strip_clear(strip_handle_);
}


// 🔵 [개선] 상태 표시: 처리 중 (파란색 숨쉬기/펄스 효과)
void LedStripDriver::set_status_busy() {
    if (!initialized_) return;
    start_effect(LedMode::BUSY);
}

// 🔴 [개선] 상태 표시: 에러 (빨간색 급박한 깜빡임 효과)
void LedStripDriver::set_status_error() {
    if (!initialized_) return;
    start_effect(LedMode::ERROR);
}

// 🟢 상태 표시: 정상 (안정적인 녹색 상시 점등)
void LedStripDriver::set_status_ok() {
    if (!initialized_) return;
    stop_current_effect();
    set_pixel(0, 0, 20, 0); // 은은한 녹색
    update();
}

// 🟡 상태 표시: 경고 (노란색 느린 깜빡임)
void LedStripDriver::set_status_warning() {
    if (!initialized_) return;
    start_effect(LedMode::WARNING);
}


void LedStripDriver::init_sequence()
{
  if (!initialized_) return;
    const uint32_t delay_ms = 200;
    set_pixel(0, 20,  0,  0); update(); vTaskDelay(pdMS_TO_TICKS(delay_ms));
    set_pixel(0,  0, 20,  0); update(); vTaskDelay(pdMS_TO_TICKS(delay_ms));
    set_pixel(0,  0,  0, 20); update(); vTaskDelay(pdMS_TO_TICKS(delay_ms));
    clear();
}

void LedStripDriver::led_effect_task(void *pvParameters)
{
  auto* driver = static_cast<LedStripDriver*>(pvParameters);
    uint32_t tick_count = 0;
    bool toggle = false;

    while (true) {
        if (driver->current_mode_ == LedMode::BUSY) {
            // 🌊 파란색 숨쉬기(Breathing) 효과: 0 ~ 40 밝기를 부드럽게 왕복
            // 사인파(Sin)를 이용하여 각도를 라디안으로 계산 (약 2초 주기)
            float angle = (tick_count % 100) * (2.0f * M_PI / 100.0f);
            uint32_t brightness = static_cast<uint32_t>((sinf(angle) + 1.0f) * 20.0f); // 0 ~ 40 범위
            
            driver->set_pixel(0, 0, 0, brightness);
            driver->update();
            vTaskDelay(pdMS_TO_TICKS(20)); // 20ms마다 밝기 갱신 (부드러운 50fps)
            tick_count++;
        }
        else if (driver->current_mode_ == LedMode::ERROR) {
            // 🚨 빨간색 스트로보 효과: 150ms 간격으로 빠르게 깜빡임
            toggle = !toggle;
            if (toggle) {
                driver->set_pixel(0, 80, 0, 0); // 밝은 빨간색
            } else {
                driver->clear();
            }
            driver->update();
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        else if (driver->current_mode_ == LedMode::WARNING) {
            // ⚠️ 노란색 일반 깜빡임: 500ms 간격으로 느리게 깜빡임
            toggle = !toggle;
            if (toggle) {
                driver->set_pixel(0, 40, 30, 0); // 노란색 (R+G)
            } else {
                driver->clear();
            }
            driver->update();
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// 현재 실행 중인 백그라운드 효과 태스크를 안전하게 종료
void LedStripDriver::stop_current_effect()
{
    if (blink_task_handle_ != nullptr) {
        vTaskDelete(blink_task_handle_);
        blink_task_handle_ = nullptr;
    }
    current_mode_ = LedMode::OFF;
}

void LedStripDriver::start_effect(LedMode mode)
{
    stop_current_effect();
    current_mode_ = mode;
    // xTaskCreate를 통해 백그라운드에서 LED 효과 구동 (우선순위는 낮게 설정)
    xTaskCreatePinnedToCore(LedStripDriver::led_effect_task, "led_effect_task", 2048, this, 2, &blink_task_handle_,0);
}

} // namespace Drivers
