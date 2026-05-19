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


namespace Controller {

esp_err_t Flight::initialize(){
    return ESP_OK;
}

esp_err_t Flight::deinitialize(){
    return ESP_OK;
}

void Flight::flight_task(void *pvParameters)
{
    // 태스크 워치독(TWDT) 등록
    esp_task_wdt_add(nullptr);
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


    Utils::SharedDataManager& sharedData = Utils::SharedDataManager::getinstance();

    // 공유 메모리 매니저 및 칼만 필터 싱글톤 인스턴스 획득
    // 1. 싱글톤 인스턴스 가져오기 및 초기화
    // 파라미터 순서: Q_gyro, Q_angle, R_accel, R_mag
    // (드론의 진동 특성에 따라 이 노이즈 파라미터들을 미세 조정합니다)
    Filter::KalmanFilter& kalman = Filter::KalmanFilter::getInstance();
    kalman.init(0.0005f, 0.005f, 0.4f, 1.0f);

    
    Service::EspNow&  espnow = Service::EspNow::get_instance();
    if (sensorTask == nullptr) {
        ESP_LOGE(TAG, "치명적 오류: EspNow 인스턴스 생성 실패!");
        vTaskDelete(nullptr);
        return;
    }
    espnow.initialize();
    espnow.connect_callback();

    uint32_t loop_cnt = 0;
    int64_t last_time = esp_timer_get_time();    
    ImuData cur_imu_data {};
    ESP_LOGI(TAG, "Flight 제어 태스크가 Core 1에서 가동되었습니다.");
    while (true) {
        // [단계 B] 시간 변화량(dt) 초정밀 계산 (초 단위 변환)
        int64_t current_time = esp_timer_get_time();
        float dt = static_cast<float>(current_time - last_time) / 1'000'000.0f;
        last_time = current_time;
        // 워치독 피딩 (루프 정상 구동 인증)
        esp_task_wdt_reset(); 

        // [단계 A] Core 0이 채워놓은 가장 최신의 정제된 IMU 데이터를 안전하게 복사 가동 (Mutex 내장)
        sharedData.get_latest_imu(cur_imu_data);
        
        //calibration이 될때까지 진입 금지........
        if (!sharedData.is_imu_calibrated()) continue; 

        kalman.update(  cur_imu_data.acc, 
                        cur_imu_data.gyro * DEG_TO_RAD, 
                        cur_imu_data.mag, 
                        dt
                    ); // 내부적으로 mag 보정 프로세스를 자동으로 거침
        Vector3f EulerDeg = kalman.getEuler() * RAD_TO_DEG;

        //[단계 D] 과도한 UART 병목을 차단하기 위해 50ms(50Hz) 주기로만 각도 출력
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
                    EulerDeg.x,   // 이제 원복 현상 없이 정교하게 각도가 추종됩니다.
                    EulerDeg.y,
                    EulerDeg.z -7.7f,
                    dt
                );
        }            
    

        // TODO: 수집된 roll, pitch, yaw 기반으로 PID_Compute() 가동 및 DShot 모터 출력 바인딩 공간
  //      }


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



// // [단계 C] 캘리브레이션이 완료된 정상 상태일 때만 칼만 필터 갱신 및 PID 연산 진입
// if (sharedData.is_imu_calibrated()) {
//     const float DEG_TO_RAD = 0.0174532925f;
    
//     // 1. 칼만 필터 업데이트
//     Vector3f gyro_rad = current_imu_data.gyro * DEG_TO_RAD;
//     kalman.update(current_imu_data.acc, gyro_rad, current_imu_data.mag, dt);

//     // 2. 현재 추정 자세 획득 (Radian)
//     Vector3f current_attitude = kalman.getEuler();

//     // 3. 조종기 수신기 등으로부터 입력받은 목표 희망 자세 (Radian 가정)
//     Vector3f target_attitude = { 0.0f, 0.0f, 0.0f }; // 예: 호버링 목표

//     // 4. [핵심] PID 입력용 오차(Error) 계산 및 부호 체계 동기화
//     Vector3f pid_error;
    
//     // Roll: Y동쪽 내릴때(+) 사양 -> 오차 정상 작동
//     pid_error.x = target_attitude.x - current_attitude.x; 
    
//     // Pitch: X북쪽 내릴때(-) 사양 -> 💡 복원 제어 부호 반전을 위해 마이너스(-) 강제 적용
//     pid_error.y = -(target_attitude.y - current_attitude.y); 
    
//     // Yaw: 우회전(+) 사양 -> 오차 정상 작동
//     pid_error.z = target_attitude.z - current_attitude.z; 

//     // 5. 각도 불연속면 경계선 정규화 (-PI ~ +PI)
//     auto normalize_error = [](float& err) {
//         while (err >  M_PI) err -= 2.0f * M_PI;
//         while (err < -M_PI) err += 2.0f * M_PI;
//     };
//     normalize_error(pid_error.x);
//     normalize_error(pid_error.y);
//     normalize_error(pid_error.z);

//     // 6. 이 정형화된 pid_error를 PID 함수에 그대로 주입하면 부호 문제 없이 작동합니다!
//     // float roll_out  = PID_Compute(&pid_roll,  pid_error.x, dt);
//     // float pitch_out = PID_Compute(&pid_pitch, pid_error.y, dt);
//     // float yaw_out   = PID_Compute(&pid_yaw,   pid_error.z, dt);
// }
