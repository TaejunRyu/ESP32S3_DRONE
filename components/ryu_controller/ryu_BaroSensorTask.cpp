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
    float currentFilteredAlt{}, lastFilteredAlt{};
    float climbRate{};
    float altOffset{};       
    bool isFirstAltCalculated = false; 

    float previousPressure{};
    float gnd_pressure{};
    float pressure{};
    BaroData baro_buf {};

    // 최종 융합 고도 상태 보존용 전역형 변수 초기화
    float gpsFusedAlt = 0.0f; 
    float gps_base_altitude = 0.0f;
    bool is_gps_home_set = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

    while (true) {
        if (bmp388.is_data_ready()){
            esp_err_t err = bmp388.get_pressure(&pressure);
            if(err == ESP_OK){
                if (pressure < 900){
                    pressure = previousPressure;
                }
                previousPressure = pressure;

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
                    
                    if (!isFirstAltCalculated) {
                        altOffset = currentFiltered;
                        isFirstAltCalculated = true;
                    }
                    
                    // 오프셋이 반영된 완전무결한 순수 상대 고도 확정
                    currentFilteredAlt = currentFiltered - altOffset;
                    
                    // 속도 변화율 연산 (dt = 0.020s)
                    float raw_rate = (currentFilteredAlt - lastFilteredAlt) / 0.020f; 
                    
                    // 승강률 끈적한 로우패스 필터
                    climbRate = (climbRate * 0.95f) + (raw_rate * 0.05f);
                    
                    // 1차적으로 기압계 기반 고도 동기화
                    lastFilteredAlt = currentFilteredAlt;                    
                } else {
                    currentFilteredAlt = 0.0f;
                    climbRate = 0.0f;
                    lastFilteredAlt = 0.0f;
                }
                
                // 🛠️ [버그 정정 1] GPS가 안 도는 루프에서도 이전 융합 누적본이 
                // 안전하게 기본값으로 복사되도록 순서를 완벽하게 격리 방어합니다.
                gpsFusedAlt = currentFilteredAlt;

                // 3. GPS 데이터 이벤트 수신 및 상보 필터 융합
                // gps_data_t mGps{};
                // if(SharedDataManager::getInstance().is_gps_updated()){
                //     mGps = SharedDataManager::getInstance().get_shared_data<Data_type::DT_GPS_DATA>();
                    
                //     if (mGps.fixType >= 3) { // 3D Fix 이상 신뢰 수준 확보 시
                        
                //         if (!is_gps_home_set && cal_gndPressure) {
                //             gps_base_altitude = mGps.horMSL; 
                //             is_gps_home_set = true;
                //         }
                        
                //         if (is_gps_home_set) {
                //             // GPS 기준 상대 고도 추출
                //             float gpsRelativeAlt = mGps.horMSL - gps_base_altitude;

                //             // 장기 날씨 드리프트 감쇄용 99.8% : 0.2% 상보 필터 가동
                //             gpsFusedAlt = (currentFilteredAlt * 0.998f) + (gpsRelativeAlt * 0.002f);
                            
                //             // 🛠️ [버그 정정 2] 물리 파이프라인 대통합 동기화
                //             // GPS 융합 결과를 기압계 고도 제어선들에도 똑같이 피딩해주어야 
                //             // 다음 루프 승강률(raw_rate) 계산 시 고도가 뚝뚝 끊기며 수직 점프하는 현상이 원천 차단됩니다.
                //             currentFilteredAlt = gpsFusedAlt;
                //             lastFilteredAlt = gpsFusedAlt; 
                //         }
                //     }
                // }

                // 4. 최종 정렬된 데이터를 갱신 발행
                baro_buf.climb_rate     = climbRate;
                baro_buf.gnd_pressure   = gnd_pressure;
                baro_buf.pressure       = pressure;
                baro_buf.altitude       = gpsFusedAlt; // 제어 루프에 공급되는 완벽한 융합 고도
                baro_buf.timestamp      = esp_timer_get_time();
                
                SharedDataManager::getInstance().publish_data<Data_type::DT_BARO_DATA>(baro_buf);
                SharedDataManager::getInstance().set_baro_updated(true);
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
