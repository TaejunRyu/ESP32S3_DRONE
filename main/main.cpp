#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>



extern "C"  void app_main(void)
{

    ESP_LOGI("MAIN","시작....");

    while(true){


        vTaskDelay( pdMS_TO_TICKS(1000));
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