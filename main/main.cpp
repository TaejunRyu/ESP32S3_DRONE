#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_FlightTask.hpp"

static const char* TAG = "DRONE";


extern "C"  void app_main(void){

    ESP_LOGI(TAG, "=== 드론 하드웨어 HAL 테스트 시작 ===");
    Control::Flight _flight;
    _flight.start_task();


    while(true){
        vTaskDelay( pdMS_TO_TICKS(100));
    }
}
