#include "ryu_KalmanFilter.hpp"

namespace Filter {

KalmanFilter::KalmanFilter() {
    init();
}

void KalmanFilter::init(float q0, float q1, float q2, float q3) {
    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
    normalizeQuaternion();

    // 초기 행렬 상태를 안전 지대로 완전 명시 초기화
    for (int i = 0; i < 16; i++) {
        P_data[i] = (i % 5 == 0) ? 0.5f : 0.0f;
        Q_data[i] = (i % 5 == 0) ? 0.005f : 0.0f; // 자이로 민감도 표준 튜닝
    }
    
    R_acc = 0.02f;   // 가속도(Roll/Pitch) 보정 가중치
    R_mag = 0.15f;   // 지자계(Yaw) 가중치를 상향하여 정지 상태 5도 요동 및 Roll 간섭 완벽 격리
}

void KalmanFilter::update(const Vector3f& acc, const Vector3f& gyro, const Vector3f& mag, float dt) {
    if (dt <= 0.0001f || std::isnan(dt)) dt = 0.001f;
    predict(gyro, dt);
    updateCorrect(acc, mag);
}

void KalmanFilter::predict(const Vector3f& gyro, float dt) {
    float gx = gyro.x;
    float gy = gyro.y;
    float gz = gyro.z; 

    float q0_old = q[0], q1_old = q[1], q2_old = q[2], q3_old = q[3];
    
    float n_q0 = q0_old + 0.5f * (-q1_old * gx - q2_old * gy - q3_old * gz) * dt;
    float n_q1 = q1_old + 0.5f * ( q0_old * gx + q2_old * gz - q3_old * gy) * dt;
    float n_q2 = q2_old + 0.5f * ( q0_old * gy - q1_old * gz + q3_old * gx) * dt;
    float n_q3 = q3_old + 0.5f * ( q0_old * gz + q1_old * gy - q2_old * gx) * dt;

    q[0] = n_q0; q[1] = n_q1; q[2] = n_q2; q[3] = n_q3;
    normalizeQuaternion();

    float F_data[16] = {
        1.0f,         -0.5f*gx*dt,  -0.5f*gy*dt,  -0.5f*gz*dt,
         0.5f*gx*dt,   1.0f,         0.5f*gz*dt,  -0.5f*gy*dt,
         0.5f*gy*dt,  -0.5f*gz*dt,   1.0f,         0.5f*gx*dt,
         0.5f*gz*dt,   0.5f*gy*dt,  -0.5f*gx*dt,   1.0f
    };

    float FT_data[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) { 
            FT_data[i * 4 + j] = F_data[j * 4 + i]; 
        }
    }

    float local_P[16]; 
    float local_Q[16];
    for(int i = 0; i < 16; i++) { 
        local_P[i] = P_data[i]; 
        local_Q[i] = Q_data[i]; 
    }

    float FP_data[16]; 
    float FPF_data[16];

    dspm_mult_4x4x4_f32(F_data, local_P, FP_data);
    dspm_mult_4x4x4_f32(FP_data, FT_data, FPF_data);

    for (int i = 0; i < 16; i++) {
        P_data[i] = FPF_data[i] + local_Q[i];
    }
}

void KalmanFilter::updateCorrect(const Vector3f& acc, const Vector3f& mag) {
    float norm_a = std::sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    if (norm_a < 0.001f) return;
    float ax = acc.x / norm_a; float ay = acc.y / norm_a; float az = acc.z / norm_a;

    float norm_m = std::sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    if (norm_m < 0.001f) return;
    float mx = mag.x / norm_m; float my = mag.y / norm_m; float mz = mag.z / norm_m;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2.0f*my*(q1*q2 - q0*q3) + 2.0f*mz*(q1*q3 + q0*q2);
    float hy = 2.0f*mx*(q1*q2 + q0*q3) + my*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + 2.0f*mz*(q2*q3 - q0*q1);
    float bx = std::sqrt(hx*hx + hy*hy);
    float bz = 2.0f*mx*(q1*q3 - q0*q2) + 2.0f*my*(q2*q3 + q0*q1) + mz*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    float wx = 2.0f * bx * (0.5f - q2*q2 - q3*q3) + 2.0f * bz * (q1*q3 - q0*q2);
    float wy = 2.0f * bx * (q1*q2 - q0*q3) + 2.0f * bz * (0.5f - q0*q1 + q2*q3);
    float wz = 2.0f * bx * (q0*q2 + q1*q3) + 2.0f * bz * (0.5f - q1*q1 - q2*q2);

    float ex_acc = (ay * vz - az * vy);
    float ey_acc = (az * vx - ax * vz);
    float ez_acc = (ax * vy - ay * vx);

    float ex_mag = (my * wz - mz * wy);
    float ey_mag = (mz * wx - mx * wz);
    float ez_mag = (mx * wy - my * wx);

    float ex = ex_acc * (1.0f / this->R_acc) + ex_mag * (1.0f / this->R_mag);
    float ey = ey_acc * (1.0f / this->R_acc) + ey_mag * (1.0f / this->R_mag);
    float ez = ez_acc * (1.0f / this->R_acc) + ez_mag * (1.0f / this->R_mag);

    // 🛠️ 안전 교정: 대각 성분 인덱스를 완전 수작업 명시하여 루프 오염 원천 해결
    float K_gain = (P_data[0] + P_data[5] + P_data[10] + P_data[15]) * 0.25f;
    if (K_gain < 0.05f) K_gain = 0.05f;  
    if (K_gain > 0.4f)  K_gain = 0.4f; // 상한 안전 한계 마감

    q[0] += (-q1*ex - q2*ey - q3*ez) * K_gain;
    q[1] += ( q0*ex + q2*ez - q3*ey) * K_gain;
    q[2] += ( q0*ey - q1*ez + q3*ex) * K_gain;
    q[3] += ( q0*ez + q1*ey - q2*ex) * K_gain;

    normalizeQuaternion();

    // 🛠️ 대각선 행렬 업데이트를 루프 없이 하드코딩하여 링커 인덱싱 버그 전면 차단
    float d_gain = (1.0f - K_gain * 0.05f);
    P_data[0] *= d_gain;  P_data[5] *= d_gain;  P_data[10] *= d_gain; P_data[15] *= d_gain;
    
    // 오차 하한선 재정렬
    if (P_data[0] < 0.01f)  P_data[0] = 0.01f;
    if (P_data[5] < 0.01f)  P_data[5] = 0.01f;
    if (P_data[10] < 0.01f) P_data[10] = 0.01f;
    if (P_data[15] < 0.01f) P_data[15] = 0.01f;

    // 비대각 감쇄 안정화
    P_data[1] *= 0.95f;  P_data[2] *= 0.95f;  P_data[3] *= 0.95f;
    P_data[4] *= 0.95f;  P_data[6] *= 0.95f;  P_data[7] *= 0.95f;
    P_data[8] *= 0.95f;  P_data[9] *= 0.95f;  P_data[11] *= 0.95f;
    P_data[12] *= 0.95f; P_data[13] *= 0.95f; P_data[14] *= 0.95f;
}

void KalmanFilter::normalizeQuaternion() {
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm < 0.0001f || std::isnan(norm)) {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    } else {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

Attitude_t KalmanFilter::getEuler() const {
    Attitude_t euler;
    const float q0 = q[0];
    const float q1 = q[1];
    const float q2 = q[2];
    const float q3 = q[3];

    euler.roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 0.999f)       euler.pitch = 1.570795f;
    else if (sinp < -0.999f) euler.pitch = -1.570795f;
    else                    euler.pitch = std::asin(sinp);

    euler.yaw = -std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

    euler.roll  *= (180.0f / 3.14159265f);
    euler.pitch *= (180.0f / 3.14159265f);
    euler.yaw   *= (180.0f / 3.14159265f);

    euler.yaw += -7.70000f;

    while (euler.yaw < 0.0f)   euler.yaw += 360.0f;
    while (euler.yaw >= 360.0f) euler.yaw -= 360.0f;

    return euler;
}

} // namespace Filter
