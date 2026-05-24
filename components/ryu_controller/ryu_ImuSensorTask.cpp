#include "ryu_ImuSensorTask.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ryu_Types.hpp"
#include "ryu_spi.hpp"
#include "ryu_i2c.hpp"
#include "ryu_ICM20948.hpp"
#include "ryu_IST8310.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_BusInterface.hpp"
#include "ryu_KalmanFilter.hpp"
#include "ryu_FlightTask.hpp"


namespace Controller {

esp_err_t ImuSensorTask::initialize()
{
    esp_err_t err = ESP_OK;
    if (_initialized) return err;

    if(!Driver::SPI::getInstance().is_initialized()){
        Driver::SPI::getInstance().initialize();
    }

    Interface::IBus* imu_interface = Interface::createBIF(Driver::SPI::getInstance().get_host(), SPI_CS_PIN);

    if(!Sensor::ICM20948::getInstance().is_initialized()){
        Sensor::ICM20948::getInstance().set_bus(imu_interface);
        Sensor::ICM20948::getInstance().initialize();
        Sensor::ICM20948::getInstance().enable_mag_bypass();
        Sensor::ICM20948::getInstance().set_include_mag(true);  // ak09916포함
    }

    //Sensor::ICM20948::getInstance().calibration_mag_hard_iron();


    if(!SharedDataManager::getInstance().is_initialized()){
        SharedDataManager::getInstance().initialize();
    }



    _initialized = true;
    return err;
}

void ImuSensorTask::readImuSensorTask(void* pvParameters) {
    ESP_LOGI(TAG, "ImuSensor가 Core %d에서 시작되었습니다.", xPortGetCoreID());
    ESP_LOGI(TAG, "센서 0점 교정을 시작합니다. 기체를 평평한 곳에 두고 움직이지 마세요.");
 
    ImuSensorTask* task = static_cast<ImuSensorTask*>(pvParameters);
    auto& icm20948 = Sensor::ICM20948::getInstance();
  
    int communication_fail_count = 0; 
    int cal_sample_count = 0;
    
    esp_err_t err = ESP_OK;
    // 1kHz 주기 제어 설정 (1ms)
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 
    TickType_t xLastWakeTime = xTaskGetTickCount();     
    while (true) {
        SensorData imu_data {};

        // [버그 패치] 호출 주체를 칩 하위 주체 대신 내장된 updateSample 인터페이스로 복원
        err = icm20948.updateSample(imu_data);
        
        if (err == ESP_OK) {
            communication_fail_count = 0; // 통신 성공 시 무조건 최상단에서 실패 카운트 리셋!

            // 0점 교정이 아직 완료되지 않은 경우는 데이터를 중계하지 않고 필터 학습 진행
            if (!icm20948.is_calibration()) {
                icm20948.calibration_loop(imu_data, ++cal_sample_count);
                
                // 교정 중인 상태 로그 출력 (지나친 로그 방지를 위해 100번에 한 번씩)
                if (cal_sample_count % 100 == 0) {
                    ESP_LOGI(TAG, "센서 교정 중... (%d / %d)", cal_sample_count, icm20948.CALIBRATION_COUNT);
                }                
                if (icm20948.is_calibration()) {
                    SharedDataManager::getInstance().set_imu_calibrated(true);
                    ESP_LOGI(TAG, "센서 0점 교정 완료! 정상 데이터 수집 및 필터링을 시작합니다.");
                }
            } 
            else {
                // 0점 교정이 끝난 실전 비행 모드 데이터 정제 작업
                icm20948.apply_filter(imu_data);
            }
            icm20948.align_NED(imu_data);            
            SharedDataManager::getInstance().publish_data<Data_type::DT_IMU_DATA>(imu_data);

            if (Flight::getInstance().getTaskHandle() != nullptr) {
                xTaskNotifyGive(Flight::getInstance().getTaskHandle());
            }
        } 
        else {
            communication_fail_count++;
            ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

            // [Fail-Safe 방어 대책] 10ms 연속 먹통 시 즉각적인 비상 대책 수립
            if (communication_fail_count >= 10) {
                ESP_LOGE(TAG, "치명적 오류: IMU 연결 유실! 긴급 비상 모드 진입 필요.");
                // task->_data_manager->trigger_emergency_stop();
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void ImuSensorTask::StartTask() {
    // 비행 제어 태스크(우선순위 24)와의 배턴터치를 보장하기 위해 한 단계 낮은 우선순위 23으로 Core 0에 완벽 격리 배정
    xTaskCreatePinnedToCore(
        readImuSensorTask,             
        "readImuSensorTask",           
        4096,                   
        this,                   
        configMAX_PRIORITIES - 2, 
        &_taskHandle,                
        0                       
    );
}

} // namespace Service
