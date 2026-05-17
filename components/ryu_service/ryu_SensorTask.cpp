#include "ryu_SensorTask.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ryu_DroneTypes.hpp"
#include "ryu_spi.hpp"
#include "ryu_ICM20948.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_BusInterface.hpp"

static const char* TAG = "SensorTask";

namespace Service {

// [방어 설계] 소멸자는 인터페이스 및 자원 할당 해제 레이어가 일원화되었으므로 비워둡니다.
SensorTask::~SensorTask() {
    // 자원 해제는 태스크의 수명 주기(Life Cycle)와 일치하도록 ReadSensorTask 내부에서 완벽히 처리합니다.
}

void SensorTask::ReadSensorTask(void* pvParameters) {
    SensorTask* task = static_cast<SensorTask*>(pvParameters);

    // 1. SPI 드라이버 및 인터페이스 초기화
    Driver::SPI& spi = Driver::SPI::get_instance();
    spi.initialize();

    Interface::IBus* bus_interface = Interface::createBIF(spi.get_host(), 10);

    // [방어 설계] 동적 할당 및 NULL 포인터 검증 예외 처리
    task->_icm20948 = new (std::nothrow) Sensor::ICM20948();
    if (task->_icm20948 == nullptr) {
        ESP_LOGE(TAG, "치명적 오류: IMU 메모리 할당 실패! 시스템을 정지합니다.");
        if (bus_interface != nullptr) {
            delete bus_interface; // 생성된 인터페이스 자원 반환 필수
        }
        vTaskDelete(nullptr);
        return;
    }

    task->_icm20948->set_bus(bus_interface);
    task->_icm20948->initialize();
    task->_icm20948->enable_mag_bypass();

    task->_data_manager = &Utils::SharedDataManager::getinstance();

    ESP_LOGI(TAG, "SensorTask가 Core %d에서 시작되었습니다.", xPortGetCoreID());

    // 1kHz 주기 제어 설정 (1ms)
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 

    // 2. [방어 설계] 부팅 직후 센서 0점 교정
    int cal_sample_count = 0;
    ESP_LOGI(TAG, "센서 0점 교정을 시작합니다. 기체를 평평한 곳에 두고 움직이지 마세요.");
    
    xLastWakeTime = xTaskGetTickCount(); 

    while (cal_sample_count < 500) {
        ImuData raw_data {};
        
        if (task->_icm20948->updateSample(raw_data) == ESP_OK) {
            task->_icm20948->calibration_loop(raw_data, ++cal_sample_count);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    ESP_LOGI(TAG, "센서 0점 교정 완료. 정상 데이터 수집 루프로 진입합니다.");

    // 3. 실전 비행 데이터 초고속 수집 무한 루프
    int communication_fail_count = 0; 
    xLastWakeTime = xTaskGetTickCount(); 

    while (true) {
        ImuData imu_data {};

        esp_err_t err = task->_icm20948->updateSample(imu_data);

        if (err == ESP_OK) {
            communication_fail_count = 0; 
            task->_icm20948->apply_filter(imu_data);
            task->_data_manager->update_latest_imu(imu_data);
        } 
        else {
            communication_fail_count++;
            ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

            if (communication_fail_count >= 10) {
                ESP_LOGE(TAG, "치명적 오류: IMU 연결 유실! 긴급 비상 모드 진입 필요.");
                // TODO: 비상 비행 중단 함수 호출 배치
                // break; // 비상 시 루프를 탈출하여 하단 자원 해제 코드로 안전 유도 가능
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    // [방어 설계] 만약 무한 루프를 빠져나올 경우를 대비한 안전한 자원 해제 레이어
    delete task->_icm20948;
    task->_icm20948 = nullptr;
    delete bus_interface;
    vTaskDelete(nullptr);
}

void SensorTask::StartTask() {
    xTaskCreatePinnedToCore(
        ReadSensorTask,             
        "ReadSensorTask",           
        4096,                   
        this,                   
        configMAX_PRIORITIES - 1, 
        nullptr,                
        0                       
    );
}

} // namespace Service
