#include "ryu_BaroSensorTask.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "ryu_Types.hpp"
#include "ryu_spi.hpp"
#include "ryu_BMP388.hpp"
#include "ryu_BusInterface.hpp"
#include "ryu_SharedDataManager.hpp"

namespace Controller {


esp_err_t BaroSensorTask::initialize()
{
    if (!Driver::SPI::getInstance().is_initialized()){
        Driver::SPI::getInstance().initialize();
    }
    
    if (!Sensor::BMP388::getInstance().is_initialized()){
        Interface::IBus* spi_interface = Interface::createBIF(Driver::SPI::getInstance().get_host(), SPI_CS_PIN);
        Sensor::BMP388::getInstance().set_bus(spi_interface);
        Sensor::BMP388::getInstance().initialize();
    }

    return ESP_OK; // 🛠️ 구현되지 않은 상태 방지
}


void BaroSensorTask::ReadBaroSensorTask(void* pvParameters) {
    auto& bmp388 = Sensor::BMP388::getInstance();

    bool cal_gndPressure = false;
    float sumPressure{0};
    uint16_t sumCount{0};
    
    float currentAlt{0.0f};
    float rawFilteredAlt{0.0f};   // [수정] 오프셋 적용 전, 필터링만 거친 절대 고도
    float currentFilteredAlt{0.0f};
    float altOffset{0.0f};       
    bool isFirstAltCalculated = false; 

    float gnd_pressure{0.0f};   
    float pressure{0.0f};       
    BaroData baro_buf {};

    size_t communication_fail_count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40); 

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 정주기 보장

        esp_err_t err = bmp388.get_pressure(&pressure);
        if (err == ESP_OK) {
            communication_fail_count = 0;

            // 1. 초기 100샘플(약 4초) 동안 지면 기압 평균 산출
            if (!cal_gndPressure) {
                sumPressure += pressure;
                ++sumCount;
                if (sumCount >= 100) { 
                    gnd_pressure = sumPressure / static_cast<float>(sumCount);
                    cal_gndPressure = true;       
                    ESP_LOGI(TAG, "Ground Pressure Calibration Success! Base: %.3f hPa", gnd_pressure);
                }
            } 

            // 2. 지면 기압 확정 후 고도 연산
            if (cal_gndPressure) {
                // 표준 대기압 공식 기반 절대 고도 계산
                currentAlt = 44330.0f * (1.0f - powf(pressure / gnd_pressure, 0.190295f));
                
                // [수정] 오프셋이 반영되지 않은 순수 기압 고도 상태에서 LPF(Low Pass Filter) 적용
                if (!isFirstAltCalculated) {
                    rawFilteredAlt = currentAlt; // 최초 실행 시 필터 초기값 지정
                    altOffset = currentAlt;      // 최초 고도를 오프셋(기준점)으로 저장
                    isFirstAltCalculated = true;
                } else {
                    // 40ms 주기 기준의 컷오프 필터 연산
                    rawFilteredAlt = (currentAlt * 0.2f) + (rawFilteredAlt * 0.8f);
                }
                
                // [수정] 필터링이 완료된 절대 고도에서 오프셋을 차감하여 완전한 '상대 고도' 확정
                currentFilteredAlt = rawFilteredAlt - altOffset;
                                    
            } else {
                currentFilteredAlt = 0.0f;
            }
            
            // 4. 최종 데이터 발행
            baro_buf.gnd_pressure   = gnd_pressure;
            baro_buf.pressure       = pressure;
            baro_buf.altitude       = currentFilteredAlt; 
            baro_buf.timestamp      = esp_timer_get_time();
            
            SharedDataManager::getInstance().publish_data<Data_type::DT_BARO_DATA>(baro_buf);
            SharedDataManager::getInstance().set_baro_updated(true);

        } else {
            communication_fail_count++;
            ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

            if (communication_fail_count >= 10) {
                ESP_LOGE(TAG, "치명적 오류: Baro 연결 유실! 긴급 비상 모드 진입 필요.");
                // SharedDataManager::getInstance().trigger_emergency_stop();
            }
        }
    }
}

void BaroSensorTask::StartTask() {
    xTaskCreatePinnedToCore(
        ReadBaroSensorTask,
        "ReadBaroSensorTask",
        3072,
        this,
        configMAX_PRIORITIES - 3, // SENSOR_TASK보다 낮게 설정하여 우선순위 밀림 원천 차단
        &_taskHandle,
        0                         // 통신 버스 분담을 위해 Core 0번에 배치
    );
}


} // namespace Controller
