#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_FlightTask.hpp"

static const char* TAG = "DRONE";


extern "C"  void app_main(void)
{
    ESP_LOGI(TAG, "=== 드론 하드웨어 HAL 테스트 시작 ===");
    Control::Flight _flight;
    _flight.start_task();


    while(true){
        vTaskDelay( pdMS_TO_TICKS(100));
    }

}



// main.cpp
// #include "ICM20948_SPI.hpp"
// #include "SensorTask.hpp"
// #include "DroneState.hpp"

// extern "C" void app_main() {
//     // 1. 전역 상태 및 센서 객체를 Stack 또는 Static 영역에 단 '하나'만 생성
//     static DroneState droneState;
//     static ICM20948_SPI imuSensor(SPI2_HOST, GPIO_NUM_10); 

//     // 2. 주소만 넘겨주어(주입) 필요한 곳에서 전역으로 공유하게 함
//     static SensorTask sensorTask(imuSensor, droneState);
    
//     sensorTask.start(); // FreeRTOS 태스크 구동
// }