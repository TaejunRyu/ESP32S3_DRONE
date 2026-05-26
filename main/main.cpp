#include <stdio.h>
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_FlightTask.hpp"
#include "ryu_ledstrip.hpp"

static const char* TAG = "DRONE";

extern "C"  void app_main(void);
void check_system_health_on_boot(void);


void app_main(void){
    check_system_health_on_boot();
    // 2. LED 스트립 초기화 및 시작 시퀀스
    auto& led_strip = Drivers::LedStripDriver::getInstance();
    led_strip.initialize();
    led_strip.init_sequence();




    //-----------WIFI 초기화----------
    // NVS 초기화
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        nvs_flash_erase();
        nvs_flash_init();
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Event loop Creation Error.");
    }


    ESP_LOGI(TAG, "=== 드론 하드웨어 HAL 테스트 시작 ===");
    Controller::Flight& flight = Controller::Flight::getInstance();
    err = flight.initialize();
    if (err!=ESP_OK){
        ESP_LOGE(TAG, "Flight Task Initialize Failed.");
        vTaskDelete(flight.getTaskHandle());
    }
    err = flight.StartTask();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Flight Task Start Failed.");
        vTaskDelete(flight.getTaskHandle());
    }else{
        ESP_LOGI(TAG, "Entering main loop...");
    }


    while(true){
        vTaskDelay( pdMS_TO_TICKS(100));
        led_strip.init_sequence();
    }
    flight.deinitialize();
    ESP_LOGI(TAG, "System shutdown complete");
}


// 시스템 상태 확인
void check_system_health_on_boot(void) {
    ESP_LOGI(TAG, "=== System Health Check ===");
    ESP_LOGI(TAG, "Free heap: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %u bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM: %u bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Total PSRAM: %u bytes", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
}