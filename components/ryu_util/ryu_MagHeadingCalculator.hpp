#pragma once
#include <cmath>
#include "DroneTypes.hpp"

namespace Utils {

class MagHeadingCalculator {
public:
    /**
     * @brief 기울어진 드론의 상태를 반영하여 지자계 기반의 정확한 절대 방위각(Yaw)을 계산합니다.
     * @param mag ENU 또는 칩 기준 정렬이 완료된 지자계 마이크로테슬라(uT) 벡터 데이터
     * @param roll_rad 현재 실시간 추정된 Roll 각도 (라디안)
     * @param pitch_rad 현재 실시간 추정된 Pitch 각도 (라디안)
     * @return float 북쪽 기준의 절대 방위각 (라디안, -PI ~ +PI 범위)
     */
    static float calculate_tilt_compensated_heading(const Vector3f& mag, float roll_rad, float pitch_rad) {
        // 삼각함수 연산 최적화
        float cos_r = std::cos(roll_rad);
        float sin_r = std::sin(roll_rad);
        float cos_p = std::cos(pitch_rad);
        float sin_p = std::sin(pitch_rad);

        // [기울기 보정 수식] 지자계 벡터를 가상 수평면(Earth Frame)으로 변환
        float mag_x_horiz = mag.x * cos_p + mag.y * sin_p * sin_r + mag.z * sin_p * cos_r;
        float mag_y_horiz = mag.y * cos_r - mag.z * sin_r;

        // 보정된 수평 벡터를 기준으로 방위각(Yaw) 도출
        // 지자계 센서와 기체 전방 매핑에 따라 부호가 바뀔 수 있으므로 기본 방향은 atan2(-mag_y, mag_x) 형태를 따름
        float heading = std::atan2(-mag_y_horiz, mag_x_horiz);

        // [중요: 편각 보정 (Declination)] 
        // 대한민국(서울/순cheon 기준 2026년 현재)은 진북과 자북의 차이가 약 -8.5도(라디안: -0.148rad)입니다.
        // 완벽한 네비게이션을 원한다면 지역 편각 값을 더해줍니다.
        // heading += (-8.5f * M_PI / 180.0f);

        // -PI ~ PI 범위 제한
        while (heading > M_PI)  heading -= (2.0f * M_PI);
        while (heading < -M_PI) heading += (2.0f * M_PI);

        return heading;
    }
};

} // namespace Utils



// Utils::KalmanFilter kalman_roll;
// Utils::KalmanFilter kalman_pitch;
// Utils::KalmanFilter kalman_yaw; // ★ Yaw 전용 칼만 엔진 추가

// void flight_control_loop(float dt) {
//     ImuData imu_data;
//     icm20948.updateSample(imu_data); 
//     calibrator.apply_filter(imu_data); 
    
//     // ENU 축 정렬 (가속도, 자이로, 지자계 모두 ENU로 정렬되었다고 가정)
//     Utils::FrameTransformer::convert_to(imu_data, Utils::TargetCoord::ENU);

//     // [1단계] Roll, Pitch 칼만 필터 업데이트 (기존과 동일)
//     float acc_roll  = std::atan2(imu_data.acc.y, imu_data.acc.z);
//     float acc_pitch = std::atan2(-imu_data.acc.x, std::sqrt(imu_data.acc.y * imu_data.acc.y + imu_data.acc.z * imu_data.acc.z));

//     float estimated_roll_enu  = kalman_roll.update(acc_roll, imu_data.gyro.x, dt);
//     float estimated_pitch_enu = kalman_pitch.update(acc_pitch, imu_data.gyro.y, dt);

//     // [2단계] ★ 지자계 데이터를 활용한 Yaw 보정 레이어 가동
//     // 방금 구한 깨끗한 roll, pitch 실시간 각도를 주입하여 지자계 틸트 보정 수행
//     float mag_heading_enu = Utils::MagHeadingCalculator::calculate_tilt_compensated_heading(
//         _mag, // ICM20948 내부에서 읽어온 지자계 raw 데이터 구조체 변수
//         estimated_roll_enu, 
//         estimated_pitch_enu
//     );

//     // Yaw축 칼만 필터 업데이트 (자이로 Z축 각속도 + 지자계 보정 각도 융합)
//     // 자이로의 빠른 회전 반응을 따라가면서, 지자계가 나침반처럼 중심을 꽉 잡아주어 드리프트가 0이 됩니다.
//     float estimated_yaw_enu = kalman_yaw.update(mag_heading_enu, imu_data.gyro.z, dt);

//     // [3단계] 최종 ENU 각도 취합 후 드론 표준 NED로 변환
//     Utils::Attitude enu_attitude = {estimated_roll_enu, estimated_pitch_enu, estimated_yaw_enu};
//     Utils::Attitude ned_attitude = Utils::AngleTransformer::enu_to_ned(enu_attitude, Utils::AngleUnit::RADIAN);

//     // [4단계] 모터 제어 엔진으로 전달
//     // pid.compute(ned_attitude);
// }
