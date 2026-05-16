/**
 * @file ryu_AngleTranformer.hpp
 * @author your name (you@domain.com)
 * @brief 
 *      1. EULER 각도를 ENU와 NED로 서로 변경할때 사용.
 * @version 0.1
 * @date 2026-05-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include <cmath>
#include "ryu_DroneTypes.hpp"
namespace Utils {

// 각도 단위 정의
enum class AngleUnit { RADIAN, DEGREE };


class AngleTransformer {
private:
    static constexpr const char* TAG = "AngleTransformer";        
public:
    /**
     * @brief ENU 각도 체계를 드론 표준 NED 각도 체계로 변환합니다.
     * @param enu 변환 전 ENU 기반 각도 구조체
     * @param unit 입력 데이터의 단위 (라디안 또는 디그리)
     * @return Attitude 변환 완료된 NED 기반 각도 구조체
     */
    static Attitude enu_to_ned(const Attitude& enu, AngleUnit unit) {
        Attitude ned = {};

        if (unit == AngleUnit::DEGREE) {
            // 1. Degree 변환 (90 - Yaw 공식 적용)
            ned.roll  = enu.roll;
            ned.pitch = -enu.pitch;
            ned.yaw   = 90.0f - enu.yaw;

            // 2. Yaw 각도 범위를 -180도 ~ +180도 사이로 보정 (Angle Wrapping)
            ned.yaw = wrap_180(ned.yaw);
        } 
        else {
            // 3. Radian 변환 (90도는 라디안으로 M_PI / 2.0f)
            ned.roll  = enu.roll;
            ned.pitch = -enu.pitch;
            ned.yaw   = (M_PI / 2.0f) - enu.yaw;

            // 4. Yaw 각도 범위를 -PI ~ +PI 사이로 보정
            ned.yaw = wrap_pi(ned.yaw);
        }

        return ned;
    }

private:
    // Degree 단위 -180 ~ 180도 범위 제한 함수
    static float wrap_180(float angle) {
        while (angle > 180.0f)  angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }

    // Radian 단위 -PI ~ PI 범위 제한 함수
    static float wrap_pi(float angle) {
        while (angle > M_PI)  angle -= (2.0f * M_PI);
        while (angle < -M_PI) angle += (2.0f * M_PI);
        return angle;
    }
};

} // namespace Utils


// void control_loop() {
//     // 1. 센서 칩에서 raw 정규 데이터 수신
//     ImuData imu_raw;
//     icm20948.updateSample(imu_raw);

//     // 2. 가속도/자이로 벡터 축만 정렬하여 필터에 주입
//     // ... 필터 연산 후 ENU 기반의 결과 각도가 나왔다고 가정 ...
//     Utils::Attitude estimated_enu = {current_roll, current_pitch, current_yaw};

//     // 3. 드론 비행 제어 알고리즘(PID)이 NED 기반이라면 진입 전 변환!
//     Utils::Attitude control_ned = Utils::AngleTransformer::enu_to_ned(estimated_enu, Utils::AngleUnit::RADIAN);

//     // 4. 이제 안전하게 무인이동체 표준 축과 부호로 모터 제어 연산 수행
//     // pid_update(control_ned.roll, control_ned.pitch, control_ned.yaw);
// }
