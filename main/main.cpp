#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BusInterface.hpp"
#include "Icm20948.hpp"
#include "VirtualSendor.hpp"
#include "spi.hpp"
#include "DroneState.hpp"

static const char* TAG = "MAIN_TEST";


extern "C"  void app_main(void)
{
    ESP_LOGI(TAG, "=== 드론 하드웨어 HAL 테스트 시작 ===");

    DroneState droneState;
    


    // 1. 최하단 하드웨어 버스 드라이버 초기화
    Driver::SPI::get_instance().initialize();

    // 2. 상위 센서 제어 객체 생성
    Sensor::ICM20948 icm20948;

    // 3. 중간 인터페이스 계층(BIF)을 팩토리 함수로 생성하여 센서에 주입
    // SPI2 호스트 사용, CS 핀 번호 2번 가정
    Interface::IBus* bus_interface = Interface::createBIF(Driver::SPI::get_instance().get_host(), 2);
    
    if (bus_interface == nullptr) {
        ESP_LOGE(TAG, "인터페이스 생성 실패! 하드웨어 등록 에러.");
        return;
    }

    // 센서에 통신 버스 주입 (수정하신 내역 반영됨)
    icm20948.set_bus(bus_interface);
    ESP_LOGI(TAG, "센서 객체에 SPI 통신 인터페이스 주입 성공.");

    // 4. 센서 칩 초기화 (Who Am I 체크, 가속도/자이로 설정 등)
    if (icm20948.initialize() == ESP_OK) {
        ESP_LOGI(TAG, "ICM20948 센서 연결 및 초기화 성공!");
    } else {
        ESP_LOGE(TAG, "ICM20948 초기화 실패! 하드웨어 핀이나 주소를 확인하세요.");
    }

    // 5. 무한 루프 데이터 샘플링 테스트
    ESP_LOGI(TAG, "데이터 리딩 루프 진입...");
    //int test_count = 0;


    Sensor::VirtualIMU vimu;
    vimu.initialize();


    while(true){
        ImuData data{};
        vimu.updateSample(data);
        ESP_LOGI("MAIN","AX: %10.5f AY: %10.5f AZ: %10.5f",data.acc.x,data.acc.y,data.acc.z);

        
        
        
        //droneState.setImuData(data);
    

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