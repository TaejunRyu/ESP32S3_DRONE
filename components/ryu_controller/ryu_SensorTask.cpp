#include "ryu_SensorTask.hpp"

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



namespace Service {

SensorTask::~SensorTask() {
    // 자원 해제는 태스크의 수명 주기와 일치하도록 내부에서 완벽히 처리합니다.
}

esp_err_t SensorTask::updateSample(ImuData &sample)
{
    ImuData data {}; // 임시 버퍼 초기화
    data.acc = 0.0f;
    data.gyro = 0.0f;
    data.mag = 0.0f; 

    // 1. 칩 레지스터 일괄 리딩 (내부에서 mag 데이터 및 플래그 갱신됨)
    esp_err_t err = _icm20948->read_data(data); 
    
    // 2. 통신이 완벽하게 성공한 경우에만 상위 객체로 데이터 복사
    if (err == ESP_OK){
        // [기본 IMU 데이터 전달]
        if (_icm20948->is_calibration()){
            sample.acc         = data.acc   - _icm20948->get_acc_bias();
            sample.gyro        = data.gyro  - _icm20948->get_gyro_bias();
        } else {
            sample.acc         = data.acc;
            sample.gyro        = data.gyro;
        }
        sample.temperature = data.temperature;
        sample.timestamp   = data.timestamp;

        if (data.is_mag_updated){
            sample.mag = (data.mag - _icm20948->get_mag_offset()) * _icm20948->get_mag_scale();
            sample.mag_timestamp  = data.mag_timestamp;
            sample.is_mag_updated = data.is_mag_updated;
            _icm20948->set_mag_previous(sample.mag);   // 정상으로 읽었을 때 자료 보관
        } else {
            sample.mag = _icm20948->get_mag_previous(); // 읽지 못하였을 경우 이전값 복원
        }     
    }
    return err;
}

void SensorTask::ReadSensorTask(void* pvParameters) {
    ESP_LOGI(TAG, "SensorTask가 Core %d에서 시작되었습니다.", xPortGetCoreID());

    SensorTask* task = static_cast<SensorTask*>(pvParameters);

    // 1. SPI 드라이버 및 인터페이스 초기화
    Driver::SPI& spi = Driver::SPI::get_instance();
    spi.initialize();

    //Driver::I2C& i2c = Driver::I2C::get_instance();
    //i2c.initialize();
    
    Interface::IBus* spi_interface = Interface::createBIF(spi.get_host(), SPI_CS_PIN);
    //Interface::IBus* i2c_interface = Interface::createBIF(i2c.get_bus_handle(),Sensor::IST8310::ADDR);


    // 동적 할당 및 NULL 포인터 검증 예외 처리
    task->_icm20948 = new (std::nothrow) Sensor::ICM20948();
    if (task->_icm20948 == nullptr) {
        ESP_LOGE(TAG, "치명적 오류: IMU 메모리 할당 실패! 시스템을 정지합니다.");
        if (spi_interface != nullptr) {
            delete spi_interface; 
        }
        vTaskDelete(nullptr);
        return;
    }

    task->_icm20948->set_bus(spi_interface);
    task->_icm20948->initialize();
    task->_icm20948->enable_mag_bypass();
    task->_icm20948->set_include_mag(true);  // ak09916포함


    //task->_ist8310->set_bus(i2c_interface);
    //task->_ist8310->initialize();



    // 싱글톤 중계 데이터 매니저 포인터 바인딩 완료
    task->_data_manager = &Utils::SharedDataManager::getinstance();

    // 1kHz 주기 제어 설정 (1ms)
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 

    // 2. 부팅 직후 센서 0점 교정 안내
    ESP_LOGI(TAG, "센서 0점 교정을 시작합니다. 기체를 평평한 곳에 두고 움직이지 마세요.");
    
    int communication_fail_count = 0; 
    int cal_sample_count = 0;
    xLastWakeTime = xTaskGetTickCount(); 
    
    // 3. 실전 비행 데이터 초고속 수집 및 캘리브레이션 무한 루프
    while (true) {
        ImuData imu_data {};
        
        // [버그 패치] 호출 주체를 칩 하위 주체 대신 내장된 updateSample 인터페이스로 복원
        esp_err_t err = task->_icm20948->updateSample(imu_data);
        
        if (err == ESP_OK) {
            communication_fail_count = 0; // 통신 성공 시 무조건 최상단에서 실패 카운트 리셋!

            // 0점 교정이 아직 완료되지 않은 경우는 데이터를 중계하지 않고 필터 학습 진행
            if (!task->_icm20948->is_calibration()) {
                task->_icm20948->calibration_loop(imu_data, ++cal_sample_count);
                
                // 교정 중인 상태 로그 출력 (지나친 로그 방지를 위해 100번에 한 번씩)
                if (cal_sample_count % 100 == 0) {
                    ESP_LOGI(TAG, "센서 교정 중... (%d / %d)", cal_sample_count, task->_icm20948->CALIBRATION_COUNT);
                }
                
                if (task->_icm20948->is_calibration()) {
                    task->_data_manager->set_imu_calibrated(true);
                    ESP_LOGI(TAG, "센서 0점 교정 완료! 정상 데이터 수집 및 필터링을 시작합니다.");
                }
            } 
            else {
                // 0점 교정이 끝난 실전 비행 모드 데이터 정제 작업
                task->_icm20948->apply_filter(imu_data);
                task->_icm20948->align_NED(imu_data);
                
                // [치명적 버그 수정] 물리적 크기 보존을 위해 여기서 mag.normalize()를 강제 수행하던 연산 제거
                
                // [중계자 복사] 뮤텍스 락 오버헤드가 제거된 고속 대입 채널 전송
                task->_data_manager->update_latest_imu(imu_data);

                // [초고속 저지연 파이프라인] 데이터 준비가 완료되었으므로 Core 1에서 대기 중인 비행 태스크를 즉시 무오래 깨움
                TaskHandle_t flight_handle = task->_data_manager->get_flight_task_handle();
                if (flight_handle != nullptr) {
                    xTaskNotifyGive(flight_handle);
                }
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
    
    // 자원 해제 레이어
    delete task->_icm20948;
    //delete task->_ist8310;
    task->_icm20948 = nullptr;
    //task->_ist8310  = nullptr;
    delete spi_interface;
    //delete i2c_interface;
    vTaskDelete(nullptr);
}

void SensorTask::StartTask() {
    // 비행 제어 태스크(우선순위 24)와의 배턴터치를 보장하기 위해 한 단계 낮은 우선순위 23으로 Core 0에 완벽 격리 배정
    xTaskCreatePinnedToCore(
        ReadSensorTask,             
        "ReadSensorTask",           
        4096,                   
        this,                   
        configMAX_PRIORITIES - 2, 
        nullptr,                
        0                       
    );
}

} // namespace Service
