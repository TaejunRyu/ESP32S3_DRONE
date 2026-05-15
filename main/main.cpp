#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BusInterface.hpp"
#include "Icm20948.hpp"
#include "VirtualSendor.hpp"
#include "spi.hpp"


extern "C"  void app_main(void)
{
    Driver::SPI::get_instance().initialize();
    


    Sensor::VirtualIMU vimu;
    vimu.initialize();
    


    ESP_LOGI("MAIN","시작....");
    ImuData data;
  
    while(true){
        vimu.updateSample(data);
        ESP_LOGI("MAIN","AX: %10.5f AY: %10.5f AZ: %10.5f",data.acc.x,data.acc.y,data.acc.z);

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