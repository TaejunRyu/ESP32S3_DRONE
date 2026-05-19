#include <stdio.h>
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_FlightTask.hpp"

static const char* TAG = "DRONE";


extern "C"  void app_main(void){


    //-----------WIFI 초기화----------
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "=== 드론 하드웨어 HAL 테스트 시작 ===");
    Controller::Flight _flight;
    _flight.start_task();


    while(true){
        vTaskDelay( pdMS_TO_TICKS(100));
    }
}
