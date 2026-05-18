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

SensorTask::~SensorTask() {
    // 자원 해제는 태스크의 수명 주기와 일치하도록 내부에서 완벽히 처리합니다.
}

void SensorTask::ReadSensorTask(void* pvParameters) {
    SensorTask* task = static_cast<SensorTask*>(pvParameters);

    // 1. SPI 드라이버 및 인터페이스 초기화
    Driver::SPI& spi = Driver::SPI::get_instance();
    spi.initialize();

    Interface::IBus* bus_interface = Interface::createBIF(spi.get_host(), SPI_CS_PIN);

    // 동적 할당 및 NULL 포인터 검증 예외 처리
    task->_icm20948 = new (std::nothrow) Sensor::ICM20948();
    if (task->_icm20948 == nullptr) {
        ESP_LOGE(TAG, "치명적 오류: IMU 메모리 할당 실패! 시스템을 정지합니다.");
        if (bus_interface != nullptr) {
            delete bus_interface; 
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

    // 2. 부팅 직후 센서 0점 교정 안내
    ESP_LOGI(TAG, "센서 0점 교정을 시작합니다. 기체를 평평한 곳에 두고 움직이지 마세요.");
    
    int communication_fail_count = 0; 
    int cal_sample_count = 0;
    xLastWakeTime = xTaskGetTickCount(); 
    uint32_t loop_cnt = 0;
    // 3. 실전 비행 데이터 초고속 수집 및 캘리브레이션 무한 루프
    while (true) {
        ImuData imu_data {};
        esp_err_t err = task->_icm20948->updateSample(imu_data);
        
        if (err == ESP_OK) {
            communication_fail_count = 0; // 💡 통신 성공 시 무조건 최상단에서 실패 카운트 리셋!

            // 0점 교정이 아직 완료되지 않은 경우
            if (!task->_icm20948->is_calibration()) {
                task->_icm20948->calibration_loop(imu_data, ++cal_sample_count);
                
                // 💡 [선택적 방어 코드] 교정 중인 로그 출력 (지나친 로그 방지를 위해 100번에 한 번씩)
                if (cal_sample_count % 100 == 0) {
                    ESP_LOGI(TAG, "센서 교정 중... (%d / 1000)", cal_sample_count);
                }
                
                if (task->_icm20948->is_calibration()) {
                    task->_data_manager->set_imu_calibrated(true);
                    ESP_LOGI(TAG, "센서 0점 교정 완료! 정상 데이터 수집 및 필터링을 시작합니다.");
                }
            } 
            else {
                // 0점 교정이 끝난 실전 비행 모드 데이터 정제 작업
                task->_icm20948->apply_filter(imu_data);
            }
            // 💡 [좌표 정렬 및 데이터 토스 일원화] 
            // 캘리브레이션 중이더라도 제어단(Core 1)이 굶지 않고 루프를 돌 수 있도록 데이터를 계속 던져줍니다.
            imu_data.mag.norm();
            task->_icm20948->align_NED(imu_data);
            task->_data_manager->update_latest_imu(imu_data);

            // if (++loop_cnt >= 50) { 
            //     loop_cnt = 0;
            //     ESP_LOGI("RAW_CHECK", "ACC=> X: %6.3f | Y: %6.3f | Z: %6.3f || GYRO=> X: %6.3f | Y: %6.3f | Z: %6.3f || MAG=> X: %6.3f | Y: %6.3f | Z: %6.3f", 
            //         imu_data.acc.x,  imu_data.acc.y,  imu_data.acc.z,
            //         imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z,
            //         imu_data.mag.x, imu_data.mag.y, imu_data.mag.z);
            // }
        } 
        else {
            communication_fail_count++;
            ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

            // [Fail-Safe 방어 대책] 10ms 연속 먹통 시 즉각적인 비상 대책 수립
            if (communication_fail_count >= 10) {
                ESP_LOGE(TAG, "치명적 오류: IMU 연결 유실! 긴급 비상 모드 진입 필요.");
                // TODO: 비상 비행 중단 함수 호출 배치 (예: 모터 즉시 차단)
                // task->_data_manager->trigger_emergency_stop();
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    // 자원 해제 레이어
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
