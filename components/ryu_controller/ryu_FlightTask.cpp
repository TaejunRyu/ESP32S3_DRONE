#include "ryu_FlightTask.hpp"

#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "ryu_spi.hpp"
#include "ryu_ICM20948.hpp"

#include "ryu_Config.hpp"
#include "ryu_BusInterface.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_FrameTransformer.hpp"
#include "ryu_KalmanFilter.hpp"
#include "ryu_SensorTask.hpp"
#include "ryu_espnow.hpp"
#include "ryu_mavlink.hpp"
#include "ryu_timer.hpp"

namespace Controller {

esp_err_t Flight::initialize(){
    return ESP_OK;
}

esp_err_t Flight::deinitialize(){
    return ESP_OK;
}

void Flight::flight_task(void *pvParameters)
{
    esp_task_wdt_add(nullptr);
    Flight* flight = static_cast<Flight*>(pvParameters);

    Service::SensorTask* sensorTask = new (std::nothrow) Service::SensorTask();
    if (sensorTask == nullptr) {
        ESP_LOGE(TAG, "치명적 오류: SensorTask 인스턴스 생성 실패!");
        vTaskDelete(nullptr);
        return;
    }
    sensorTask->StartTask();

    Utils::SharedDataManager& sharedData = Utils::SharedDataManager::getinstance();
    Filter::KalmanFilter& kalman = Filter::KalmanFilter::getInstance();
  
    Service::EspNow& espnow = Service::EspNow::get_instance();
    espnow.initialize();
    espnow.start_task();
    espnow.connect_callback();

    Service::Timer& timer = Service::Timer::get_instance();
    timer.intiallize();
    timer.Start();

    Service::Mavlink& mavlink = Service::Mavlink::get_instance();
    mavlink.initialize();
    mavlink.start_task();
    
    uint32_t loop_cnt = 0;
    int64_t last_time = esp_timer_get_time();    
    int64_t loop_start_time = last_time; // 초정밀 주기 제어를 위한 기준 시간 변수 추가
    ImuData cur_imu_data {};
    
    ESP_LOGI(TAG, "Flight 제어 태스크가 Core 1에서 가동되었습니다.");
    
    while (true) {
        // [단계 B] 시간 변화량(dt) 초정밀 계산 (초 단위 변환)
        int64_t current_time = esp_timer_get_time();
        float dt = static_cast<float>(current_time - last_time) / 1'000'000.0f;
        last_time = current_time;
        
        loop_start_time = current_time; // 루프가 시작된 절대 시점 확보
        
        esp_task_wdt_reset(); 

        // [단계 A] Core 0이 수집한 IMU 데이터 안전 복사
        sharedData.get_latest_imu(cur_imu_data);
        
        // 센서 캘리브레이션 완료 대기
        if (!sharedData.is_imu_calibrated()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            last_time = esp_timer_get_time(); // 대기하는 동안 벌어진 dt 오차 누적 방지
            continue; 
        }

        // 축 매핑 및 스케일 팩터 동기화 업데이트 진행
        kalman.update(cur_imu_data.acc, 
                      cur_imu_data.gyro * DEG_TO_RAD, 
                      cur_imu_data.mag, dt);
                      
        Attitude_t attitude = kalman.getEuler();

        // QGC 및 Mavlink 데이터 버퍼 전송
        mavlink._attitude.roll  = attitude.roll;
        mavlink._attitude.pitch = attitude.pitch;
        mavlink._attitude.yaw   = attitude.yaw;
        mavlink._attitude.roll_speed  = cur_imu_data.gyro.x * RAD_TO_DEG;
        mavlink._attitude.pitch_speed = cur_imu_data.gyro.y * RAD_TO_DEG;
        mavlink._attitude.yaw_speed   = cur_imu_data.gyro.z * RAD_TO_DEG;

        // [단계 D] UART 병목 방지용 50Hz 주기 출력 루프
        if (++loop_cnt >= 50) { 
            loop_cnt = 0;
            ESP_LOGI(TAG, "|AX: %8.5f |AY: %8.5f |AZ: %8.5f |GX: %8.5f |GY: %8.5f |GZ: %8.5f |MX: %8.5f |MY: %8.5f |MZ: %8.5f |R: %8.5f |P: %8.5f |Y: %8.5f  dt: %5.4f", 
                    cur_imu_data.acc.x,
                    cur_imu_data.acc.y,
                    cur_imu_data.acc.z,
                    cur_imu_data.gyro.x * DEG_TO_RAD,
                    cur_imu_data.gyro.y * DEG_TO_RAD,
                    cur_imu_data.gyro.z * DEG_TO_RAD,
                    cur_imu_data.mag.x,
                    cur_imu_data.mag.y,
                    cur_imu_data.mag.z,
                    attitude.roll,   
                    attitude.pitch,
                    attitude.yaw, // 내부 수식에서 -7.7f가 이미 처리되었으므로 중복 제거 완료
                    dt
                );
        }            

        // TODO: PID 제어 연산 및 모터 출력 바인딩 공간

        // [단계 E] 초정밀 주기 제어 + 워치독 방어 구조 최적화
        int64_t wake_time = esp_timer_get_time();
        int64_t elapsed = wake_time - loop_start_time;
        
        // 여유 시간이 200us 이상 많이 남았을 때만 IDLE 릴리즈 수행
        if (LOOP_TIME - elapsed > 200) {
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }

        // 1ms 미만의 미세 잔여 지터는 정확한 루프 시작 시점 기준(loop_start_time)으로 폴링 마감
        while (true) {
            if (esp_timer_get_time() - loop_start_time >= LOOP_TIME) {
                break; 
            }
        }
    }

    delete sensorTask;
    vTaskDelete(nullptr);
}

void Flight::start_task()
{
    xTaskCreatePinnedToCore(
        flight_task,                
        "flight_task",              
        8192, 
        this,                       
        configMAX_PRIORITIES - 1,   
        nullptr,                    
        1 
    );
}

} // namespace Controller
