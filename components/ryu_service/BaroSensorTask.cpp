#include "BaroSensorTask.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ryu_DroneTypes.hpp"
#include "Icm20948.hpp"
#include "SharedDataManager.hpp"

static const char* TAG = "SENSOR_TASK";

namespace Service {

void BaroTask::ReadBaroTask(void* pvParameters) {
    BaroTask* task = static_cast<BaroTask*>(pvParameters);
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz = 20ms 주기 작동

    while (true) {
        // [단계 1] 센서칩에 "압력 측정 시작해라" 명령 송신 (대기 없음)
        //task->_baro_sensor->trigger_pressure_measurement();

        // [단계 2] ★ 하드웨어 변환 대기 시간 확보 (예: 9ms)
        // 이 동안 vTaskDelay가 작동하여 CPU 제어권이 최상위 권한인 'SensorTask'로 넘어가므로, 
        // 1ms IMU 리딩 타이밍에 단 1마이크로초의 방해도 주지 않습니다.
        vTaskDelay(pdMS_TO_TICKS(9));

        // [단계 3] 레지스터에서 디지털 변환이 완료된 압력 데이터 수집
        BaroData baro_buf {};
        // if (task->_baro_sensor->read_pressure_and_calc_alt(baro_buf) == ESP_OK) {
        //     baro_buf.timestamp = esp_timer_get_time();
            
        //     // [단계 4] 데이터 매니저에 안전하게 주입
        //     task->_data_manager->update_latest_baro(baro_buf);
        // }

        // 정확히 20ms 주기를 맞추기 위해 잔여 시간 휴식
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void BaroTask::StartTask() {
    xTaskCreatePinnedToCore(
        ReadBaroTask,
        "BaroTask",
        3072,
        this,
        configMAX_PRIORITIES - 3, // ★ 방어 설계: SENSOR_TASK보다 낮게 설정하여 우선순위 밀림 원천 차단
        nullptr,
        0                         // ★ 핵심: 통신 버스 분담을 위해 동일하게 Core 0번에 배치
    );
}

} // namespace Service
