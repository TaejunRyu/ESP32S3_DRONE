#pragma once

#include <cmath>
#include <algorithm>
#include <esp_log.h>
#include <esp_timer.h>
#include "ryu_DroneTypes.hpp"

namespace Filter {

class KalmanFilter {
private:
    // 싱글톤 설계를 위해 생성자 제한
    KalmanFilter();
    ~KalmanFilter() = default;
    
    static constexpr const char* TAG = "KalmanFilter";

public:
    // 복사 및 이동 금지
    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;

    static KalmanFilter& get_instance() {
        static KalmanFilter instance;
        return instance;
    }

    // 상태 벡터 x = [q0, q1, q2, q3, bx, by, bz]
    float x[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    float& gyro_x_err = x[4]; 
    float& gyro_y_err = x[5]; 
    float& gyro_z_err = x[6]; 

    // 오차 공분산 행렬 P (7x7)
    float P[7][7] = {0.0f,};

    // 필터 게인 파라미터 (S3 환경에 맞춘 튜닝값)
    const float Q_quat  = 0.001f;   // 쿼터니언 변화 노이즈
    const float Q_bias  = 0.000001f; // 자이로 바이어스 변화 노이즈
    const float R_accel = 0.05f;    // 가속도계 신뢰도 (낮을수록 가속도계를 더 믿음)

    void update(Vector3f& acc,Vector3f& gyro,Vector3f& mag, float dt);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void update(Vector3f& acc,Vector3f& gyro, float dt);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);    
    void get_euler(float* roll, float* pitch, float* yaw);
    void get_speed_err(float* rollspeed, float* pitchspeed, float* yawspeed);
    void reset();

private:
    const float R_mag = 0.2f; // 지자계 신뢰도 (주변 금속 물질이 많으면 값을 높이세요)
    void update_mag(float mx, float my, float mz);
    void predict(float gx, float gy, float gz, float dt);
    void update_accel(float ax, float ay, float az);
    void normalize_quat();
};

} // namespace Service
