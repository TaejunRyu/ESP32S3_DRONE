/**
 * @file ryu_SafeGuard.hpp
 * @author your name (you@domain.com)
 * @brief 
 *      1. 기체의 문제될수 있는 모든 값들을 체크하여 방법을 제공한다.
 * @version 0.1
 * @date 2026-05-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include <esp_err.h>
#include <inttypes.h>


namespace Controller{

enum class FlightMode {
    MANUAL,         // 일반 수동/어시스트 비행
    LANDING,        // 일반 자동 착륙
    EMERGENCY_LAND  // [추가] 센서 탈조시 비상 안전 착륙 모드
};

struct VehicleState_t {
    FlightMode mode;
    float target_roll;
    float target_pitch;
    float target_throttle;
};

class SafeGuard{
        static constexpr const char* TAG = "SafeGuard";

        // 임계값 정의 (실제 환경에 맞게 미세 조정 가능)
        static constexpr const float IMU_ERR_CRITICAL    = 0.08f;  // Roll/Pitch 오차 한계치
        static constexpr const float MAG_ERR_CRITICAL    = 0.12f;  // Yaw 오차 한계치
        static constexpr const uint32_t ERR_DURATION_MS  = 1500; // 오차가 지속되는 허용 시간 (1.5초)

        VehicleState_t _vehicle_state = { FlightMode::MANUAL, 0.0f, 0.0f, 0.5f };

    public:
        static void SafeGuard_task(void *pv);
        esp_err_t start_task();
    
};



}