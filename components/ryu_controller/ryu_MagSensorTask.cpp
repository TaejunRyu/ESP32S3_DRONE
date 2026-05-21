#include "ryu_MagSensorTask.hpp"
#include <esp_log.h>
#include "ryu_i2c.hpp"
#include "ryu_IST8310.hpp"
#include "ryu_SharedDataManager.hpp"

namespace Controller {

esp_err_t MagSensorTask::initialize() {
    if (_initialized) return ESP_OK;
    _initialized = true;
    return ESP_OK;
}

void MagSensorTask::ReadMagSensorTask(void* pvParameters) {
    ESP_LOGI("MagSensorTask", "외장 지자기(I2C) 태스크가 Core %d에서 시작되었습니다.", xPortGetCoreID());

    // 1. I2C 마스터 인스턴스 획득 및 초기화
    Driver::I2C& i2c = Driver::I2C::getInstance();
    i2c.initialize();
    
    // 하드웨어 추상화 버스 인터페이스 생성 (I2C 핸들과 IST8310 소자 주소 0x0E 결합)
    Interface::IBus* i2c_interface = Interface::createBIF(i2c.get_bus_handle(), Sensor::IST8310::ADDR);

    // 2. IST8310 센서에 I2C 버스 주입 및 장치 초기화
    auto& ist8310 = Sensor::IST8310::getInstance();
    ist8310.set_bus(i2c_interface);
    if (ist8310.initialize() != ESP_OK) {
        ESP_LOGE("MagSensorTask", "IST8310 초기화 실패! 태스크를 일시 중지합니다.");
        vTaskDelete(nullptr);
    }

    auto& data_manager = SharedDataManager::getInstance();

    // 지자기는 100Hz(10ms) 주기 기동으로 EKF 융합에 충분합니다.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 

    // 센서 데이터를 담을 로컬 구조체 (프로젝트 명세의 MagData 혹은 SensorData 타입에 맞춤)
    Vector3f mag_buf {}; 

    while (true) {
        // 3. 센서 내부 하드웨어의 변환 완료 상태 검사 (I2C 레지스터 리딩)
        if (ist8310.is_data_ready()) {
            
            // 4. 데이터 인출 및 내부 하드/소프트 아이언 오프셋 적용
            esp_err_t err = ist8310.updateSample(mag_buf); 
            
            if (err == ESP_OK) {
                // 5. 무중단 더블 버퍼 매니저에 독립 게시 (IMU 1kHz 태스크에 절대 지터를 주지 않음)
                data_manager.publish_data<Data_type::DT_MAG_DATA>(mag_buf);
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
