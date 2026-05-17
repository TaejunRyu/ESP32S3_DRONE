#include "ryu_FlightTask.hpp"

#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "ryu_spi.hpp"
#include "ryu_ICM20948.hpp"
#include "ryu_DroneConfig.hpp"
#include "ryu_BusInterface.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_FrameTransformer.hpp"
#include "ryu_KalmanFilter.h"
#include "ryu_SensorTask.hpp"

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

    // // 다른 task에서 데이터를 저장하고 또 달ㄴ 태스크에서 데이터를 읽어간다.
    // Utils::SharedDataManager& sharedData = Utils::SharedDataManager::getinstance();

    // Driver::SPI::get_instance().initialize();

    // Service::SensorTask sensorTask;
    // sensorTask.StartTask();

    // Sensor::ICM20948 _icm20948;
    
    // Interface::IBus* bus_interface = Interface::createBIF(Driver::SPI::get_instance().get_host(), 10);

    // if (bus_interface == nullptr) {
    //     ESP_LOGE(TAG, "인터페이스 생성 실패! 하드웨어 등록 에러.");
    //     return;
    // }

    // _icm20948.set_bus(bus_interface);

    // if (_icm20948.initialize() == ESP_OK) {
    //     ESP_LOGI(TAG, "ICM20948 센서 연결 및 초기화 성공!");
    // } else {
    //     ESP_LOGE(TAG, "ICM20948 초기화 실패! 하드웨어 핀이나 주소를 확인하세요.");
    // }

    // _icm20948.enable_mag_bypass();


    // //Watch Dog 등록.  
    // esp_task_wdt_add(nullptr);

    // uint32_t loop_cnt = 0;
    // int64_t  last_time = esp_timer_get_time();
    // uint16_t sample_count =0;
    // static ImuData data{};
    // data.acc =0.0f;
    // data.gyro =0.0f;
    // data.mag =0.0f;

    // //_icm20948.calibration_mag_hard_iron();



    // Filter::KalmanFilter& kalman =  Filter::KalmanFilter::get_instance();
    // kalman.reset();


    Service::SensorTask sensorTask;
    sensorTask.StartTask();

    uint32_t loop_cnt = 0;
    int64_t  last_time = esp_timer_get_time();

    while(true){
        esp_task_wdt_reset(); 

        
        // _icm20948.updateSample(data);

        // if (!_icm20948.is_calibration()){
        //     _icm20948.calibration_loop(data,++sample_count); // 자동으로 (처음 500회는 적용되지 않은 데이터가 나오다가 500부터 적용됨.
        // }
        // //_icm20948.apply_filter(data); // lpf 필터적용.
        // _icm20948.align_NED(data);
        
        // // ENU로 변환.
        // //Utils::FrameTransformer::convert_to(data,Utils::CoordSystem::ENU);

        // data.mag.norm();
        // // 4. [개선] 디버그 로그 주기를 1초(1000Hz 루프 기준 1000번)에 한 번으로 크게 변경
        // // UART 병목을 물리적으로 완전히 방지합니다.
        // if (++loop_cnt >= 50) { 
        //     loop_cnt = 0;
        //     ESP_LOGI(TAG, "| AX: %6.3f | AY: %6.3f | AZ: %6.3f | GX: %6.3f | GY: %6.3f | GZ: %6.3f| MX: %6.3f | MY: %6.3f | MZ: %6.3f|",
        //             data.acc.x, data.acc.y, data.acc.z,
        //             data.gyro.x, data.gyro.y, data.gyro.z,
        //             data.mag.x, data.mag.y, data.mag.z
        //         );
        // }
        
        // kalman.update(data.gyro,data.acc,data.mag,dt);
                    
        // float xx,yy,zz;                
        // kalman.get_euler(&xx,&yy,&zz);


        // // if (++loop_cnt >= 50) { 
        // //     loop_cnt = 0;
        // //     ESP_LOGI(TAG, "| roll: %6.3f | pitch: %6.3f | yaw: %6.3f |",
        // //              xx, yy, zz);
        // // }


        // 5. [초정밀 제어 + 워치독 방어 통합 구조]
        int64_t current_time;
        // 1kHz(1000us) 루프에서 남은 시간이 200us 이상으로 여유가 있다면,
        // 강제로 딱 1개 틱(1ms) 동안 태스크를 재워 IDLE1이 워치독을 완전히 리셋하게 만듭니다.
        current_time = esp_timer_get_time();
        if (LOOP_TIME - (current_time - last_time) > 200) {
            vTaskDelay(pdMS_TO_TICKS(1)); // 1ms 동안 스케줄러 권한 완벽 이양
        }

        // 1ms 미만의 초미세 지터(Jitter)는 아래의 타이머 폴링으로 완벽하게 마감 처리합니다.
        while (true) {
            current_time = esp_timer_get_time();
            if (current_time - last_time >= LOOP_TIME) {
                break; 
            }
        }
        last_time = current_time; 
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