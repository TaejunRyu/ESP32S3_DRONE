#include "ryu_MagSensorTask.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include "ryu_i2c.hpp"
#include "ryu_IST8310.hpp"
#include "ryu_SharedDataManager.hpp"

namespace Controller {


/**
 * @brief 
 *      1. 이 지역에서 사용되어지는 모듈의 초기화 체크후 초기화 되지않았으면 초기화 한다.
 *      2. IST8310으로 데이터 처리
 * @return esp_err_t 
 */
esp_err_t MagSensorTask::initialize() {
    if (_initialized) return ESP_OK;

    if(!SharedDataManager::getInstance().is_initialized()){
        SharedDataManager::getInstance().initialize();
    }

    if(!Driver::I2C::getInstance().is_initialized()){
        Driver::I2C::getInstance().initialize();
    }
    auto i2c_interface = Interface::createBIF(Driver::I2C::getInstance().get_bus_handle(), Sensor::IST8310::ADDR);

    if(!Sensor::IST8310::getInstance().is_initialized()){
        Sensor::IST8310::getInstance().set_bus(i2c_interface);
        if(Sensor::IST8310::getInstance().initialize() != ESP_OK) {
            ESP_LOGE("MagSensorTask", "IST8310 초기화 실패! 태스크를 일시 중지합니다.");
            vTaskDelete(nullptr);
        }
    }

    _initialized = true;
    return ESP_OK;
}




void MagSensorTask::ReadMagSensorTask(void* pvParameters) {
    ESP_LOGI("MagSensorTask", "외장 지자기(I2C) 태스크가 Core %d에서 시작되었습니다.", xPortGetCoreID());

    auto& ist8310 = Sensor::IST8310::getInstance();
    
    Vector3f mag_buf {}; 
    size_t communication_fail_count = 0;
    uint32_t log_loop_cnt = 0;       
    
    // 시간 측정을 위한 변수 (정확한 주기 측정을 위해 if문 내부가 아닌 전역/스태틱 유지)
    uint64_t last_success_time_ms = esp_timer_get_time() / 1000;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz 강제

    while (true) {
        // FreeRTOS가 정확히 10ms 주기로 이 태스크를 깨웁니다.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // [개선] 타이밍 충돌을 방지하기 위해 is_data_ready() 조건문을 제거하고 
        // 10ms 주기마다 무조건 센서 데이터를 업데이트(I2C Read) 요청합니다.
        esp_err_t err = ist8310.updateSample(mag_buf); 
        
        if (err == ESP_OK) {
            ist8310.align_NED(mag_buf);
            
            SharedDataManager::getInstance().publish_data<Data_type::DT_MAG_DATA>(mag_buf);
            SharedDataManager::getInstance().set_mag_updated(true);
            communication_fail_count = 0; 

            // --- [수정] 정확한 시간 간격 측정 로직 ---
            // log_loop_cnt++;
            // if (log_loop_cnt >= 100) { // 100번 성공할 때마다 (약 1초마다) 주기 출력
            //     uint64_t current_time_ms = esp_timer_get_time() / 1000;
            //     // 100번 돌았으므로 평균 간격은 (현재시간 - 1초전시간) / 100이 되어야 합니다.
            //     float average_interval = static_cast<float>(current_time_ms - last_success_time_ms) / 100.0f;
                
            //     ESP_LOGI(TAG, "지자기 센서 100회 평균 읽기 주기: %.2f ms", average_interval);
                
            //     last_success_time_ms = current_time_ms; // 출력이 일어난 시점에만 기준 시간 갱신
            //     log_loop_cnt = 0;
            // }

        } else {
            communication_fail_count++;
            ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

            if (communication_fail_count >= 10) {
                // 주석에는 Baro라고 되어있으나 Mag 태스크이므로 에러 로그 수정
                ESP_LOGE(TAG, "치명적 오류: 지자기(Mag) 센서 연결 유실!");
                // SharedDataManager::getInstance().trigger_emergency_stop();
            }
        }
    }
}

void MagSensorTask::StartTask() {
    xTaskCreatePinnedToCore(
        ReadMagSensorTask,
        "ReadMagSensorTask",
        3072,
        this,
        configMAX_PRIORITIES - 4, // 우선순위: IMU(MAX-2) 및 Baro(MAX-3)보다 낮게 배정하여 밀림 차단
        &_taskHandle,
        0                         // I2C 버스 자원 관리를 위해 수집 전용 Core 0번에 격리 배정
    );    
}


} // namespace Controller
