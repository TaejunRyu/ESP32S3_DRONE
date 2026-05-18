#include "ryu_KalmanFilter.hpp"

namespace Filter {

KalmanFilter::KalmanFilter()
    : Q_angle_(0.005f), Q_gyro_(0.0005f), R_accel_(0.4f), R_mag_(1.0f) { // 반응 속도 최적화 파라미터 기본 적용
    
    estimated_attitude_ = {0.0f, 0.0f, 0.0f};
    bias_ = {0.0f, 0.0f, 0.0f};

    Px00 = 1.0f; Px01 = 0.0f; Px10 = 0.0f; Px11 = 1.0f;
    Py00 = 1.0f; Py01 = 0.0f; Py10 = 0.0f; Py11 = 1.0f;
    Pz00 = 1.0f; Pz01 = 0.0f; Pz10 = 0.0f; Pz11 = 1.0f;
}

void KalmanFilter::init(float q_gyro, float q_angle, float r_accel, float r_mag) {
    std::lock_guard<std::mutex> lock(mtx_);
    Q_gyro_ = q_gyro;
    Q_angle_ = q_angle;
    R_accel_ = r_accel;
    R_mag_ = r_mag;
}

Vector3f KalmanFilter::getEuler() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return estimated_attitude_;
}

void KalmanFilter::normalizeVector(Vector3f& v) {
    float norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm > 0.001f) {
        v.x /= norm; v.y /= norm; v.z /= norm;
    } else {
        v.x = 0.0f; v.y = 0.0f; v.z = 1.0f;
    }
}

void KalmanFilter::update(const Vector3f& acc, const Vector3f& gyro_rad, const Vector3f& mag, float dt) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    // -------------------------------------------------------------------------
    // 1. 상태 및 공분산 예측 (State & Covariance Prediction)
    // 지정 사양 동기화: Roll (+), Pitch (-), Yaw (+) 각속도 맵핑 완료
    // -------------------------------------------------------------------------
    float rate_roll  = gyro_rad.x;
    float rate_pitch = gyro_rad.y;  // X축 내릴때 Pitch(-) 연산을 위해 정부호 입력 유지 (가속도 측정축과 동기화)
    float rate_yaw   = gyro_rad.z;

    // 자이로 적분
    estimated_attitude_.x += (rate_roll  - bias_.x) * dt;
    estimated_attitude_.y += (rate_pitch - bias_.y) * dt;
    estimated_attitude_.z += (rate_yaw   - bias_.z) * dt;

    // 공분산 예측
    Px00 += dt * (dt * Px11 - Px01 - Px10 + Q_angle_); Px01 -= dt * Px11; Px10 -= dt * Px11; Px11 += Q_gyro_ * dt;
    Py00 += dt * (dt * Py11 - Py01 - Py10 + Q_angle_); Py01 -= dt * Py11; Py10 -= dt * Py11; Py11 += Q_gyro_ * dt;
    Pz00 += dt * (dt * Pz11 - Pz01 - Pz10 + Q_angle_); Pz01 -= dt * Pz11; Pz10 -= dt * Pz11; Pz11 += Q_gyro_ * dt;

    // -------------------------------------------------------------------------
    // 2. 가속도계 데이터 측정 보정 (Accel Measurement Update)
    // -------------------------------------------------------------------------
    Vector3f norm_acc = acc;
    normalizeVector(norm_acc);

    // 지정 사양: X 내릴 때 PITCH (-) 가 되도록 가속도 부호 조정
    float measured_pitch = std::atan2(norm_acc.x, std::sqrt(norm_acc.y * norm_acc.y + norm_acc.z * norm_acc.z));
    // 지정 사양: Y 내릴 때 ROLL (+) 가 되도록 설정
    float measured_roll  = std::atan2(norm_acc.y, norm_acc.z);

    // Roll 보정
    float y_x = measured_roll - estimated_attitude_.x;
    float S_x = Px00 + R_accel_;
    float K_x0 = Px00 / S_x; float K_x1 = Px10 / S_x;
    estimated_attitude_.x += K_x0 * y_x; bias_.x += K_x1 * y_x;
    float Px00_tmp = Px00; float Px01_tmp = Px01;
    Px00 -= K_x0 * Px00_tmp; Px01 -= K_x0 * Px01_tmp; Px10 -= K_x1 * Px00_tmp; Px11 -= K_x1 * Px01_tmp;

    // Pitch 보정
    float y_y = measured_pitch - estimated_attitude_.y;
    float S_y = Py00 + R_accel_;
    float K_y0 = Py00 / S_y; float K_y1 = Py10 / S_y;
    estimated_attitude_.y += K_y0 * y_y; bias_.y += K_y1 * y_y;
    float Py00_tmp = Py00; float Py01_tmp = Py01;
    Py00 -= K_y0 * Py00_tmp; Py01 -= K_y0 * Py01_tmp; Py10 -= K_y1 * Py00_tmp; Py11 -= K_y1 * Py01_tmp;

        // -------------------------------------------------------------------------
    // 3. 지자계 데이터 경사 보정 및 Yaw 최종 추적 연산
    // -------------------------------------------------------------------------
    float cos_r = std::cos(estimated_attitude_.x);
    float sin_r = std::sin(estimated_attitude_.x);
    float cos_p = std::cos(estimated_attitude_.y);
    float sin_p = std::sin(estimated_attitude_.y);

    // Tilt Compensation (기울기 평면 투영)
    float Xh = mag.x * cos_p + mag.y * sin_p * sin_r + mag.z * sin_p * cos_r;
    float Yh = mag.y * cos_r - mag.z * sin_r;

    // [수정] 우회전(시계방향) 시 YAW 가 정상적으로 (+) 값을 가지도록 부호 전면 보정
    float measured_yaw = std::atan2(Yh, Xh); 

    // 각도 강제 정형화 오차 제어 헬퍼
    auto normalize_angle = [](float& angle) {
        while (angle > M_PI)  angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
    };
    normalize_angle(estimated_attitude_.z);

    // 추정치와 측정치의 위상 오차 계산
    float yaw_error = measured_yaw - estimated_attitude_.z;
    normalize_angle(yaw_error);

    // Yaw 축 공분산 수식 및 최종 칼만 보정 반영
    float S_z = Pz00 + R_mag_;
    float K_z0 = Pz00 / S_z; 
    float K_z1 = Pz10 / S_z;
    
    estimated_attitude_.z += K_z0 * yaw_error; 
    bias_.z               += K_z1 * yaw_error;
    
    float Pz00_tmp = Pz00; 
    float Pz01_tmp = Pz01;
    
    Pz00 -= K_z0 * Pz00_tmp; 
    Pz01 -= K_z0 * Pz01_tmp; 
    Pz10 -= K_z1 * Pz00_tmp; 
    Pz11 -= K_z1 * Pz01_tmp;

    normalize_angle(estimated_attitude_.z);

}

} // namespace Filter
