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

    auto& bmp388    = Sensor::BMP388::getInstance();

    bool cal_gndPressure = false;
    float sumPressure{0};
    uint16_t sumCount{0};
    float currentAlt{};
    float currentFilteredAlt{};
    float lastFilteredAlt{};  // 확정된 고도.
    float altOffset{};       
    bool isFirstAltCalculated = false; 

    float previousPressure{};
    float gnd_pressure{};   // 날리기전 지면의 기압.
    float pressure{};       // 현재 기압.
    BaroData baro_buf {};


    size_t communication_fail_count = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40); 

    while (true) {
        if (bmp388.is_data_ready()){
            esp_err_t err = bmp388.get_pressure(&pressure);
            if(err == ESP_OK){
                communication_fail_count = 0; // 통신 성공 시 무조건 최상단에서 실패 카운트 리셋!
                // if (pressure < 900){
                //     pressure = previousPressure;
                // }
                // previousPressure = pressure;

                // 1. 시동 시 현재 위치의 기압 수집 (평균 산출)
                if (!cal_gndPressure){
                    sumPressure += pressure;
                    ++sumCount;
                    if(sumCount >= 100){ 
                        gnd_pressure = sumPressure / (float)sumCount;
                        cal_gndPressure = true;       
                        ESP_LOGI(TAG, "Ground Pressure Calibration Success! Base: %.3f hPa", gnd_pressure);
                    }
                } 

                // 2. 그라운드 기압 기준 상대 고도 및 속도 변화율 연산
                if(cal_gndPressure){
                    currentAlt = 44330.0f * (1.0f - powf(pressure / gnd_pressure, 0.190295f));
                    float currentFiltered = (currentAlt * 0.2f) + (lastFilteredAlt * 0.8f);
                    
                    if (!isFirstAltCalculated) { // gnd_pressue으로 고도를 계산한것이 offet이되어짐.
                        altOffset = currentFiltered;
                        isFirstAltCalculated = true;
                    }                 
                    // 오프셋이 반영된 완전무결한 순수 상대 고도 확정
                    currentFilteredAlt = currentFiltered - altOffset;
                    // 1차적으로 기압계 기반 고도 동기화
                    lastFilteredAlt = currentFilteredAlt;                    
                } else {
                    currentFilteredAlt = 0.0f;
                    lastFilteredAlt = 0.0f;
                }
                
                // 4. 최종 정렬된 데이터를 갱신 발행
                baro_buf.gnd_pressure   = gnd_pressure;
                baro_buf.pressure       = pressure;
                baro_buf.altitude       = currentFilteredAlt; // 제어 루프에 공급되는 완벽한 융합 고도
                baro_buf.timestamp      = esp_timer_get_time();
                
                SharedDataManager::getInstance().publish_data<Data_type::DT_BARO_DATA>(baro_buf);
                SharedDataManager::getInstance().set_baro_updated(true);
            }else{
                communication_fail_count++;
                ESP_LOGW(TAG, "센서 통신 일시 실패 (%d회 연속)", communication_fail_count);

                // [Fail-Safe 방어 대책] 10ms 연속 먹통 시 즉각적인 비상 대책 수립
                if (communication_fail_count >= 10) {
                    ESP_LOGE(TAG, "치명적 오류: Baro 연결 유실! 긴급 비상 모드 진입 필요.");
                    // task->_data_manager->trigger_emergency_stop();
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
