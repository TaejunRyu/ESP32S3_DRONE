#include "ryu_MagSensorTask.hpp"
#include <esp_log.h>
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

    // 2. IST8310 센서에 I2C 버스 주입 및 장치 초기화
    auto& ist8310 = Sensor::IST8310::getInstance();

    //ist8310.calibrate_hard_iron();

    Vector3f mag_buf {}; 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
    while (true) {
        if (ist8310.is_data_ready()) {
            esp_err_t err = ist8310.updateSample(mag_buf); 
            if (err == ESP_OK) {
                ist8310.align_NED(mag_buf);
                //ESP_LOGW(TAG,"| mx: %8.3f | my: %8.3f | mz: %8.3f |", mag_buf.x , mag_buf.y , mag_buf.z );
                SharedDataManager::getInstance().publish_data<Data_type::DT_MAG_DATA>(mag_buf);
                SharedDataManager::getInstance().set_mag_updated(true);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
