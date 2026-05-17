#include "ryu_KalmanFilter.h"
#include <cmath>
#include <algorithm>

namespace Filter {

KalmanFilter::KalmanFilter() {
    reset();
}

void KalmanFilter::reset() {
    x[0] = 1.0f; x[1] = 0.0f; x[2] = 0.0f; x[3] = 0.0f;
    x[4] = 0.0f; x[5] = 0.0f; x[6] = 0.0f;
    
    for(int i=0; i<7; i++) {
        for(int j=0; j<7; j++) P[i][j] = 0.0f;
        P[i][i] = 0.1f;
    }
}

void KalmanFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    predict(gx, gy, gz, dt);
    update_accel(ax, ay, az);
    normalize_quat();
}

void KalmanFilter::predict(float gx, float gy, float gz, float dt) {
    // 1. 자이로 바이어스 제거
    float wx = gx - x[4];
    float wy = gy - x[5];
    float wz = gz - x[6];

    // 2. 쿼터니언 미분 방정식 (부호 오류 수정 완료)
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    x[0] += 0.5f * (-q1 * wx - q2 * wy - q3 * wz) * dt;
    x[1] += 0.5f * ( q0 * wx - q3 * wy + q2 * wz) * dt;
    x[2] += 0.5f * ( q3 * wx + q0 * wy - q1 * wz) * dt; // +q3*wx 및 부호 순서 교정
    x[3] += 0.5f * (-q2 * wx + q1 * wy + q0 * wz) * dt;

    // 3. 공분산 업데이트 및 클램핑
    for(int i=0; i<4; i++) P[i][i] += Q_quat * dt;
    for(int i=4; i<7; i++) P[i][i] += Q_bias * dt;

    for(int i=0; i<7; i++) {
        if (P[i][i] > 1.0f) P[i][i] = 1.0f;
    }
}

void KalmanFilter::update_accel(float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.5f || norm > 1.5f) return; 
    ax /= norm; ay /= norm; az /= norm;

    // [수정] NED 좌표계 표준 기준 중력 전달 벡터 (가속도계 정립 시 az = -1 또는 1 매칭)
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3; 

    // Innovation (측정 오차)
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // 칼만 이득을 통한 상태 보정 수식 안전화
    float K = 0.02f; 
    x[0] += K * (-q1 * ex - q2 * ey - q3 * ez);
    x[1] += K * ( q0 * ex + q2 * ez - q3 * ey);
    x[2] += K * ( q3 * ex + q0 * ey - q1 * ez);
    x[3] += K * (-q2 * ex + q1 * ey + q0 * ez);
    
    // 바이어스 보정
    x[4] -= K * 0.01f * ex;
    x[5] -= K * 0.01f * ey;
    x[6] -= K * 0.01f * ez;
}

void KalmanFilter::normalize_quat() {
    float n = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    if (n > 0.0f) {
        float recip = 1.0f / n;
        x[0] *= recip; x[1] *= recip; x[2] *= recip; x[3] *= recip;
    }
}

void KalmanFilter::get_euler(float* roll, float* pitch, float* yaw) {
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    //[수정] NED 시스템에 정확히 부합하는 오일러 각도 추출식
    *roll  = atan2f(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29578f;
    *pitch = asinf(std::clamp(-2.0f * (q1 * q3 - q0 * q2), -1.0f, 1.0f)) * 57.29578f;
    *yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
}

void KalmanFilter::get_speed_err(float *rollspeed, float *pitchspeed, float* yawspeed)
{
    *rollspeed  = x[4];
    *pitchspeed = x[5];
    *yawspeed   = x[6];
}

void KalmanFilter::update(Vector3f &acc, Vector3f &gyro, Vector3f &mag, float dt)
{
    predict(gyro.x, gyro.y, gyro.z, dt);
    update_accel(acc.x, acc.y, acc.z);
    update_mag(mag.x, mag.y, mag.z);
    normalize_quat();
}

void KalmanFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    predict(gx, gy, gz, dt);
    update_accel(ax, ay, az);
    update_mag(mx, my, mz);
    normalize_quat();
}

void KalmanFilter::update(Vector3f &acc, Vector3f &gyro, float dt)
{
    predict(gyro.x, gyro.y, gyro.z, dt);
    update_accel(acc.x, acc.y, acc.z);
    normalize_quat();
}

void KalmanFilter::update_mag(float mx, float my, float mz) {
    float norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < 0.1f) return;
    mx /= norm; my /= norm; mz /= norm;

    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + my * 2.0f * (q1*q2 - q0*q3) + mz * 2.0f * (q1*q3 + q0*q2);
    float hy = mx * 2.0f * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + mz * 2.0f * (q2*q3 - q0*q1);
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = mx * 2.0f * (q1*q3 - q0*q2) + my * 2.0f * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    float wx = bx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz * 2.0f * (q1*q3 - q0*q2);
    float wy = bx * 2.0f * (q1*q2 - q0*q3) + bz * 2.0f * (q0*q1 + q2*q3);
    float wz = bx * 2.0f * (q0*q2 + q1*q3) + bz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    float ex_mag = (my * wz - mz * wy);
    float ey_mag = (mz * wx - mx * wz);
    float ez_mag = (mx * wy - my * wx);

    float K = 0.01f;
    x[0] += K * (-q1 * ex_mag - q2 * ey_mag - q3 * ez_mag);
    x[1] += K * ( q0 * ex_mag + q2 * ez_mag - q3 * ey_mag);
    x[2] += K * ( q0 * ey_mag - q1 * ez_mag + q3 * ex_mag);
    x[3] += K * ( q0 * ez_mag + q1 * ey_mag - q2 * ex_mag);
    
    x[4] -= K * 0.1f * ex_mag;
    x[5] -= K * 0.1f * ey_mag;
    x[6] -= K * 0.1f * ez_mag;
}

} // namespace Filter
