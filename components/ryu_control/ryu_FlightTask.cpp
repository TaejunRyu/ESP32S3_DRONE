#include "ryu_FlightTask.hpp"

#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "ryu_spi.hpp"
#include "Icm20948.hpp"
#include "ryu_DroneConfig.hpp"
#include "ryu_BusInterface.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_FrameTransformer.hpp"
namespace Control{

esp_err_t Flight::initialize()
{
    return esp_err_t();
}

esp_err_t Flight::deinitialize()
{
    return esp_err_t();
}

void Flight::flight_task(void *pvParameters)
{
    Flight* flight = static_cast<Flight*>(pvParameters);

    // 다른 task에서 데이터를 저장하고 또 달ㄴ 태스크에서 데이터를 읽어간다.
    Utils::SharedDataManager& sharedData = Utils::SharedDataManager::getinstance();

    Driver::SPI::get_instance().initialize();

    Sensor::ICM20948 _icm20948;
    
    Interface::IBus* bus_interface = Interface::createBIF(Driver::SPI::get_instance().get_host(), 9);

    if (bus_interface == nullptr) {
        ESP_LOGE(TAG, "인터페이스 생성 실패! 하드웨어 등록 에러.");
        return;
    }

    _icm20948.set_bus(bus_interface);

    if (_icm20948.initialize() == ESP_OK) {
        ESP_LOGI(TAG, "ICM20948 센서 연결 및 초기화 성공!");
    } else {
        ESP_LOGE(TAG, "ICM20948 초기화 실패! 하드웨어 핀이나 주소를 확인하세요.");
    }

    _icm20948.enable_mag_bypass();


    //Watch Dog 등록.  
    esp_task_wdt_add(nullptr);

    uint32_t loop_cnt = 0;
    int64_t  last_time = esp_timer_get_time();
    uint16_t sample_count =0;
    static ImuData data{};
    data.acc =0.0f;
    data.gyro =0.0f;
    data.mag =0.0f;

    while(true){
        if (++loop_cnt >= 400) loop_cnt = 0; // 1초 주기로 초기화
         esp_task_wdt_reset(); 

        
        _icm20948.updateSample(data);
        _icm20948.calibration_loop(data,++sample_count); // 자동으로 (처음 500회는 적용되지 않은 데이터가 나오다가 500부터 적용됨.
        //_icm20948.apply_filter(data); // lpf 필터적용.
        _icm20948.align_NED(data);

        data.mag.norm();
        Utils::FrameTransformer::convert_to(data,Utils::CoordSystem::ENU);
        
        if(loop_cnt % 16 == 0){
            ESP_LOGI(TAG,"| AX: %9.4f | AY: %9.4f | AZ: %9.4f | GX: %9.4f | GY: %9.4f | GZ: %9.4f| MX: %9.4f | MY: %9.4f | MZ: %9.4f|",
                    data.acc.x,
                    data.acc.y,
                    data.acc.z,
                    data.gyro.x,
                    data.gyro.y,
                    data.gyro.z,
                    data.mag.x,
                    data.mag.y,
                    data.mag.z
                );
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
        
        int64_t current_time;
        while ((current_time = esp_timer_get_time()) - last_time < LOOP_TIME) {
            if (LOOP_TIME - (current_time - last_time) > 1200) {
                vTaskDelay(1); 
            }
        }
    }
}


void Flight::start_task()
{
    xTaskCreatePinnedToCore(
        flight_task,                // 실행할 함수 포인터
        "flight_task",              // 디버그용 태스크 명칭
        8192,                       // 스택 크기 (안전하게 4KB 할당)
        this,                       // 매개변수
        configMAX_PRIORITIES - 1,      // ★ 방어 설계: 시스템 최상위 우선순위 부여 (주기 왜곡 방지)
        nullptr,                    // 태스크 핸들러 변수 생략
        1                           // ★ 핵심: Core 1번 칩에 고정 바인딩 
    );
}



} // namespace Control