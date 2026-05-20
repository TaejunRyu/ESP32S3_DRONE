#include "ryu_SafeGuard.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ryu_KalmanFilter.hpp"


namespace Controller{

void SafeGuard::SafeGuard_task(void *pv)
{
    SafeGuard *safeguard = static_cast<SafeGuard *> (pv); 

    uint32_t imu_err_start_tick = 0;
    uint32_t mag_err_start_tick = 0;
    bool is_imu_fault = false;
    bool is_mag_fault = false;



    while (1) {
        // 1. EKF 싱글톤에서 실시간 공분산 오차 직접 참조
        // 헤더파일 private 구역 하단에 public 게터 함수(getXErr 등)를 두거나 직접 배열 접근 구조 활용
        float x_err = Filter::KalmanFilter::getInstance().getXErr(); // P[1][1]
        float y_err = Filter::KalmanFilter::getInstance().getYErr(); // P[2][2]
        float z_err = Filter::KalmanFilter::getInstance().getZErr(); // P[3][3]

        uint32_t current_tick = xTaskGetTickCount();

        // 2. IMU (Roll/Pitch) 오차 누적 감시
        if (x_err > SafeGuard::IMU_ERR_CRITICAL || y_err > SafeGuard::IMU_ERR_CRITICAL) {
            if (imu_err_start_tick == 0) imu_err_start_tick = current_tick;
            
            if ((current_tick - imu_err_start_tick) >= pdMS_TO_TICKS(SafeGuard::ERR_DURATION_MS)) {
                is_imu_fault = true;
            }
        } else {
            imu_err_start_tick = 0; // 오차가 낮아지면 타이머 리셋
        }

        // 3. 지자계 (Yaw) 오차 누적 감시
        if (z_err > SafeGuard::MAG_ERR_CRITICAL) {
            if (mag_err_start_tick == 0) mag_err_start_tick = current_tick;
            
            if ((current_tick - mag_err_start_tick) >= pdMS_TO_TICKS(SafeGuard::ERR_DURATION_MS)) {
                is_mag_fault = true;
            }
        } else {
            mag_err_start_tick = 0;
        }

        // 4. 비상 조치 트리거 (인터럽트 스위칭)
        if (safeguard->_vehicle_state.mode != FlightMode::EMERGENCY_LAND) {
            if (is_imu_fault) {
                ESP_LOGE(TAG, "!!! CRITICAL IMU FAULT DETECTED !!! 비상 착륙을 시작합니다.");
                safeguard->_vehicle_state.mode = FlightMode::EMERGENCY_LAND;
            } 
            else if (is_mag_fault) {
                ESP_LOGW(TAG, "!!! COMPASS FAULT DETECTED !!! 지자계 보정을 차단하고 자이로 관성 비행으로 착륙합니다.");
                // 지자계 오차일 경우 나침반 보정을 끄고(R_mag 가중치를 무한대로 올리는 효과) 안전 착륙 모드 진입
                safeguard->_vehicle_state.mode = FlightMode::EMERGENCY_LAND;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz 주기로 가볍게 감시 (CPU 점유율 최소화)
    }
}

esp_err_t SafeGuard::start_task()
{
    return esp_err_t();
}

} // namespace Controller



// void flight_control_task(void* pvParameters) {
//     while (1) {
//         // EKF 오일러 각 획득
//         Attitude_t current_attitude = Filter::KalmanFilter::getInstance().getEuler();

//         float roll_input = 0.0f;
//         float pitch_input = 0.0f;
//         float throttle_input = 0.0f;

//         // [Safe-Guard 인터럽트] 상태가 비상 착륙 모드인 경우 조종사 명령을 차단
//         if (g_vehicle_state.mode == FlightMode::EMERGENCY_LAND) {
//             // 조종사 키 입력을 강제 무시하고 수평 유지를 목표로 설정
//             g_vehicle_state.target_roll = 0.0f;   // 무조건 평평하게
//             g_vehicle_state.target_pitch = 0.0f;  // 무조건 평평하게
            
//             // 모터 출력을 현재 상태에서 매 주기마다 천천히 감산하여 부드럽게 하강 유도
//             static float emergency_throttle = 0.6f; // 비상 진입 시점의 예상 유지 스로틀
//             emergency_throttle -= 0.0005f;         // 초당 조금씩 출력 하강
//             if (emergency_throttle < 0.15f) emergency_throttle = 0.0f; // 지면에 닿으면 완전 정지
            
//             g_vehicle_state.target_throttle = emergency_throttle;
//         } else {
//             // 정상 비행 모드일 때는 수신기(RC)나 상위 내비게이션 명령을 그대로 추종
//             // g_vehicle_state.target_roll = rc_get_roll();
//         }

//         // 이 제어 목표값(target)과 EKF 출력값(current)의 에러로 PID 연산 수행
//         // run_pid_control(g_vehicle_state, current_attitude);

//         vTaskDelay(pdMS_TO_TICKS(1)); // 1kHz 초고속 제어 루프
//     }
// }
