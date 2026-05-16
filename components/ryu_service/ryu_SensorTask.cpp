#include "ryu_SensorTask.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ryu_DroneTypes.hpp"
#include "Icm20948.hpp"
#include "ryu_SharedDataManager.hpp"

static const char* TAG = "SENSOR_TASK";

namespace Service {

/**
 * @brief 센서 데이터를 1kHz(1ms) 주기로 초고속 수집 및 정제하여 공유 메모리에 주입하는 태스크
 */
void SensorTask::ReadSensorTask(void* pvParameters) {
    SensorTask* task = static_cast<SensorTask*>(pvParameters);

    ESP_LOGI(TAG, "SensorTask가 Core %d에서 시작되었습니다.", xPortGetCoreID());

    // 1. 주기 제어를 위한 FreeRTOS 변수 초기화 (1kHz = 1ms 주기)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms

    // 2. 부팅 직후 센서 정적 오프셋(Bias) 500개 샘플 수집 및 교정 루프
    int cal_sample_count = 0;
    ESP_LOGI(TAG, "센서 0점 교정을 시작합니다. 기체를 평평한 곳에 두고 움직이지 마세요.");
    
    while (cal_sample_count < 500) {
         ImuData raw_data {};
        
        // 센서 하드웨어 리딩
        if (task->_icm20948->updateSample(raw_data) == ESP_OK) {
            task->_icm20948->calibration_loop(raw_data, ++cal_sample_count);
        }
        
        // 정확히 1ms 쉬고 다음 샘플 수집
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    ESP_LOGI(TAG, "센서 0점 교정 완료. 정상 데이터 수집 루프로 진입합니다.");

    // 3. 실전 비행 데이터 초고속 수집 무한 루프
    int communication_fail_count = 0; // 하드웨어 연결 유실 방어용 카운터

    while (true) {
        ImuData imu_data {};

        // [단계 A] 센서 칩 레지스터 일괄 획득 (물리 단위 파싱 및 지자계 플래그 내부 처리됨)
        esp_err_t err = task->_icm20948->updateSample(imu_data);

        if (err == ESP_OK) {
            communication_fail_count = 0; // 에러 카운터 리셋

            // [단계 B] 방어 레이어 가동: 고주파 진동 노이즈 깎기 & 0점 오차 제거 (1차 LPF)
            task->_icm20948->apply_filter(imu_data);

            // [단계 C] 허브 매니저 주입: 새로 분리된 독립 뮤텍스를 통해 Core 1(제어단)에 안전 토스
            task->_data_manager->update_latest_imu(imu_data);
        } 
        else {
            communication_fail_count++;
            ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

            // ★ [Fail-Safe 방어 대책] 비행 중 진동이나 노이즈로 인해 연속 10회(10ms) 이상 센서가 먹통이 되면, 
            // 시스템 전역의 비상 차단 신호를 발생시키거나 모터를 즉시 안전 정지 모드로 유도해야 합니다.
            if (communication_fail_count >= 10) {
                ESP_LOGE(TAG, "치명적 오류: IMU 연결 유실! 긴급 비상 모드 진입 필요.");
                // 비상 비행 중단 함수 호출 배치 공간
            }
        }

        // [단계 D] 절대 정밀 주기 보장
        // vTaskDelay와 달리 호출 간격의 편차(Jitter)를 하드웨어 레벨에 가깝게 강제 보정하여 정확히 1ms를 유지시킵니다.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief 메인 초기화단에서 호출되어 태스크를 최고 우선순위로 Core 0에 강제 지정 바인딩하는 생성 헬퍼 함수
 */
void SensorTask::StartTask() {
    xTaskCreatePinnedToCore(
        ReadSensorTask,             // 실행할 함수 포인터
        "ReadSensorTask",           // 디버그용 태스크 명칭
        4096,                   // 스택 크기 (안전하게 4KB 할당)
        this,                   // 매개변수
        configMAX_PRIORITIES - 1, // ★ 방어 설계: 시스템 최상위 우선순위 부여 (주기 왜곡 방지)
        nullptr,                // 태스크 핸들러 변수 생략
        0                       // ★ 핵심: Core 0번 칩에 고정 바인딩 (Core 1은 PID 연산 전용으로 격리)
    );
}

} // namespace Tasks
