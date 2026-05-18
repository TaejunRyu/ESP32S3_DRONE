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
#include "ryu_KalmanFilter.hpp"
#include "ryu_SensorTask.hpp"

namespace Control {

esp_err_t Flight::initialize()
{
    return ESP_OK;
}

esp_err_t Flight::deinitialize()
{
    return ESP_OK;
}

void Flight::flight_task(void *pvParameters)
{
    Flight* flight = static_cast<Flight*>(pvParameters);

    // [방어 설계] 지역 변수 탈출: SensorTask 인스턴스를 힙 영역에 안전하게 동적 할당하여 수명 주기 보장
    Service::SensorTask* sensorTask = new (std::nothrow) Service::SensorTask();
    if (sensorTask == nullptr) {
        ESP_LOGE(TAG, "치명적 오류: SensorTask 인스턴스 생성 실패!");
        vTaskDelete(nullptr);
        return;
    }
    // Core 0번 센서 수집 스레드 강제 가동
    sensorTask->StartTask();

    // 공유 메모리 매니저 및 칼만 필터 싱글톤 인스턴스 획득
    Utils::SharedDataManager& sharedData = Utils::SharedDataManager::getinstance();
    Filter::KalmanFilter& kalman = Filter::KalmanFilter::getInstance();
    // 1. 싱글톤 인스턴스 가져오기 및 초기화
    // 파라미터 순서: Q_gyro, Q_angle, R_accel, R_mag
    // (드론의 진동 특성에 따라 이 노이즈 파라미터들을 미세 조정합니다)
    // 변경 (추천):
    kalman.init(0.0005f, 0.005f, 0.4f, 1.0f);
    //kalman.init(0.0001f, 0.001f, 0.1f, 0.5f);

    // 태스크 워치독(TWDT) 등록
    esp_task_wdt_add(nullptr);

    uint32_t loop_cnt = 0;
    int64_t last_time = esp_timer_get_time();
    
    ImuData current_imu_data {};

    ESP_LOGI(TAG, "Flight 제어 태스크가 Core 1에서 가동되었습니다.");

    while (true) {
        // 워치독 피딩 (루프 정상 구동 인증)
        esp_task_wdt_reset(); 

        // [단계 A] Core 0이 채워놓은 가장 최신의 정제된 IMU 데이터를 안전하게 복사 가동 (Mutex 내장)
        sharedData.get_latest_imu(current_imu_data); 
        
        //[단계 D] 과도한 UART 병목을 차단하기 위해 50ms(50Hz) 주기로만 각도 출력
        // if (++loop_cnt >= 50) { 
        //     loop_cnt = 0;
        //     ESP_LOGI(TAG, "|AX : %8.5f |AY : %8.5f |AZ : %8.5f |GX : %8.5f |GY : %8.5f |GZ : %8.5f |MX : %8.5f |MY : %8.5f |MZ : %8.5f |", 
        //             current_imu_data.acc.x,
        //             current_imu_data.acc.y,
        //             current_imu_data.acc.z,
        //             current_imu_data.gyro.x,
        //             current_imu_data.gyro.y,
        //             current_imu_data.gyro.z,
        //             current_imu_data.mag.x,
        //             current_imu_data.mag.y,
        //             current_imu_data.mag.z
        //         );
        // }


        // [단계 B] 시간 변화량(dt) 초정밀 계산 (초 단위 변환)
        int64_t current_time = esp_timer_get_time();
        float dt = static_cast<float>(current_time - last_time) / 1000000.0f;
        last_time = current_time;

        // [단계 C] 캘리브레이션이 완료된 정상 상태일 때만 칼만 필터 갱신 및 PID 연산 진입
        // 💡 부팅 직후 1초 동안 센서가 0점 교정 중일 때는 자세 추정 연산을 안전하게 대기시킵니다.
        if (sharedData.is_imu_calibrated()) {
            const float DEG_TO_RAD = 0.0174532925f;
             const float RAD_TO_DEG = 57.295779513f; 
            Vector3f  gyro_rad  = current_imu_data.gyro * DEG_TO_RAD;

            kalman.update(current_imu_data.acc, gyro_rad, current_imu_data.mag, dt); // 내부적으로 mag 보정 프로세스를 자동으로 거침

  
            Vector3f EulerRad = kalman.getEuler();
            
            Vector3f EulerDeg = {
                EulerRad.x * RAD_TO_DEG,
                EulerRad.y * RAD_TO_DEG,
                EulerRad.z * RAD_TO_DEG
            };

            //[단계 D] 과도한 UART 병목을 차단하기 위해 50ms(50Hz) 주기로만 각도 출력
            // if (++loop_cnt >= 50) { 
            //     loop_cnt = 0;
            //     ESP_LOGI(TAG, "[자세 상태] Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f", EulerDeg.x, EulerDeg.y, EulerDeg.z);
            // }
            //[단계 D] 과도한 UART 병목을 차단하기 위해 50ms(50Hz) 주기로만 각도 출력
            if (++loop_cnt >= 50) { 
                loop_cnt = 0;
                ESP_LOGI(TAG, "|AX : %8.5f |AY : %8.5f |AZ : %8.5f |GX : %8.5f |GY : %8.5f |GZ : %8.5f |MX : %8.5f |MY : %8.5f |MZ : %8.5f |ROLL : %8.5f |PITCH : %8.5f |YAW : %8.5f ", 
                        current_imu_data.acc.x,
                        current_imu_data.acc.y,
                        current_imu_data.acc.z,
                        current_imu_data.gyro.x * DEG_TO_RAD,
                        current_imu_data.gyro.y * DEG_TO_RAD,
                        current_imu_data.gyro.z * DEG_TO_RAD,
                        current_imu_data.mag.x,
                        current_imu_data.mag.y,
                        current_imu_data.mag.z,
                        EulerDeg.x,   // 이제 원복 현상 없이 정교하게 각도가 추종됩니다.
                        EulerDeg.y,
                        EulerDeg.z -7.7f
                    );
            }            








            // TODO: 수집된 roll, pitch, yaw 기반으로 PID_Compute() 가동 및 DShot 모터 출력 바인딩 공간
        }










        // [단계 E] 초정밀 주기 제어 + 워치독 방어 구조 유지
        int64_t wake_time = esp_timer_get_time();
        if (LOOP_TIME - (wake_time - last_time) > 200) {
            vTaskDelay(pdMS_TO_TICKS(1)); // IDLE 태스크에게 CPU 권한을 양보하여 워치독 리셋 보장
        }

        // 1ms 미만의 미세 지터는 하드웨어 타이머 폴링으로 정밀 마감
        while (true) {
            if (esp_timer_get_time() - last_time >= LOOP_TIME) {
                break; 
            }
        }
    }

    // 예외 상황 발생 시 메모리 반환
    delete sensorTask;
    vTaskDelete(nullptr);
}

void Flight::start_task()
{
    xTaskCreatePinnedToCore(
        flight_task,                
        "flight_task",              
        8192, // 연산 스택 최적화 (8KB 확보 유지)
        this,                       
        configMAX_PRIORITIES - 1,   
        nullptr,                    
        1 // Core 1번에 고정 바인딩하여 센서 수집단(Core 0)과 완벽하게 물리 격리
    );
}

} // namespace Control
