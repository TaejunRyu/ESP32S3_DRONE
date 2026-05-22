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

void BaroSensorTask::ReadBaroSensorTask(void* pvParameters) {
    // 1. SPI 드라이버 및 인터페이스 초기화
    Driver::SPI& spi = Driver::SPI::getInstance();
    spi.initialize();
    
    Interface::IBus* spi_interface = Interface::createBIF(spi.get_host(), SPI_CS_PIN);

    auto& bmp388 = Sensor::BMP388::getInstance();
    bmp388.set_bus(spi_interface);
    bmp388.initialize();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz = 20ms 주기 작동

    float filtered_alt{};
    float clib_rate{};
    float gnd_pressure {};
    BaroData baro_buf {};

    while (true) {
        //if (bmp388.is_data_ready()){
            bmp388.get_relative_altitude(&filtered_alt);
            gnd_pressure = bmp388.get_ground_pressure();
            clib_rate = bmp388.get_climb_rate();
            
            // 로컬 구조체 바인딩 (상승률 누락 수정)
            baro_buf.altitude = filtered_alt;
            baro_buf.gnd_pressure = gnd_pressure;
            baro_buf.climb_rate = clib_rate; // 🛠️ 구조체 필드가 있다면 할당 필요
            
            SharedDataManager::getInstance().publish_data<Data_type::DT_BARO_DATA>(baro_buf);
            SharedDataManager::getInstance().set_baro_updated(true);
        //}
        
        // 정확히 20ms 주기를 맞추기 위해 잔여 시간 휴식
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

esp_err_t BaroSensorTask::initialize()
{
    return ESP_OK; // 🛠️ 구현되지 않은 상태 방지
}

} // namespace Controller
