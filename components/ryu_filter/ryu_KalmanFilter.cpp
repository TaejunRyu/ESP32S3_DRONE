/**
 * @file ryu_KalmanFilter.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "ryu_KalmanFilter.hpp"
#include <cstring>

namespace Filter {

KalmanFilter::KalmanFilter() {
    Q_gyro = 0.001f;   
    R_accel = 0.10f;    // qgc의 인공선이 떨리면 0.05 ~ 0.1로 조금씩올리면 테스트 할것.(초기 성공한 값은 0.02f)
    R_mag = 0.05f;     
    init();
}

void KalmanFilter::init(float initRoll, float initPitch, float initYaw) {
    std::lock_guard<std::mutex> lock(mtx);

    float cr = std::cos(initRoll * 0.5f);
    float sr = std::sin(initRoll * 0.5f);
    float cp = std::cos(initPitch * 0.5f);
    float sp = std::sin(initPitch * 0.5f);
    float cy = std::cos(initYaw * 0.5f);
    float sy = std::sin(initYaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    normalizeQuaternion();

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = (i == j) ? 0.01f : 0.0f;
        }
    }
}

void KalmanFilter::predict(const Vector3f& gyro, float dt) {
    std::lock_guard<std::mutex> lock(mtx);

    float wx = gyro.x;
    float wy = gyro.y;
    float wz = gyro.z;

    float dq0 = 0.5f * (-q[1]*wx - q[2]*wy - q[3]*wz) * dt;
    float dq1 = 0.5f * ( q[0]*wx - q[3]*wy + q[2]*wz) * dt;
    float dq2 = 0.5f * ( q[3]*wx + q[0]*wy - q[1]*wz) * dt;
    float dq3 = 0.5f * (-q[2]*wx + q[1]*wy + q[0]*wz) * dt;

    q[0] += dq0; q[1] += dq1; q[2] += dq2; q[3] += dq3;
    normalizeQuaternion();

    float F[4][4] = {
        {1.0f, -0.5f*wx*dt, -0.5f*wy*dt, -0.5f*wz*dt},
        {0.5f*wx*dt, 1.0f, 0.5f*wz*dt, -0.5f*wy*dt},
        {0.5f*wy*dt, -0.5f*wz*dt, 1.0f, 0.5f*wx*dt},
        {0.5f*wz*dt, 0.5f*wy*dt, -0.5f*wx*dt, 1.0f}
    };

    float nextP[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                nextP[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                P[i][j] += nextP[i][k] * F[j][k];
            }
            if (i == j) P[i][j] += Q_gyro * dt;
        }
    }
}

void KalmanFilter::update(const Vector3f& accel, const Vector3f& mag) {
    std::lock_guard<std::mutex> lock(mtx);

    float a_norm = std::sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if (a_norm < 0.001f) return;
    float ax = accel.x / a_norm; float ay = accel.y / a_norm; float az = accel.z / a_norm;

    float m_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
    if (m_norm < 0.001f) return;
    
    // 크로스 축 잠금 현상을 분리하기 위해 지자계 보정 극성 완전 교정
    float mx = mag.x / m_norm; 
    float my = -mag.y / m_norm; 
    float mz = mag.z / m_norm; 

    // ==========================================
    // 1단계: 가속도계 업데이트
    // ==========================================
    float vx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    float vy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    float Ha[3][4] = {
        {-2.0f*q[2],  2.0f*q[3], -2.0f*q[0],  2.0f*q[1]},
        { 2.0f*q[1],  2.0f*q[0],  2.0f*q[3],  2.0f*q[2]},
        { 2.0f*q[0], -2.0f*q[1], -2.0f*q[2],  2.0f*q[3]}
    };

    float y_a[3] = { ax - vx, ay - vy, az - vz };
    
    float HaP[3][4] = {0};
    for(int i=0; i<3; ++i) {
        for(int j=0; j<4; ++j) {
            for(int k=0; k<4; ++k) {
                HaP[i][j] += Ha[i][k] * P[k][j];
            }
        }
    }

    float Sa[3][3] = {0};
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            for(int k=0; k<4; ++k) {
                Sa[i][j] += HaP[i][k] * Ha[j][k];
            }
            if (i == j) Sa[i][j] += R_accel;
        }
    }

    float Sa_inv[3][3];
    matrixInversion3x3(Sa, Sa_inv);

    float Ka[4][3] = {0};
    float PHaT[4][3] = {0};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<3; ++j) {
            for(int k=0; k<4; ++k) {
                PHaT[i][j] += P[i][k] * Ha[j][k];
            }
        }
    }

    for(int i=0; i<4; ++i) {
        for(int j=0; j<3; ++j) {
            for(int k=0; k<3; ++k) {
                Ka[i][j] += PHaT[i][k] * Sa_inv[k][j];
            }
        }
    }

    for(int i=0; i<4; ++i) {
        for(int j=0; j<3; ++j) {
            q[i] += Ka[i][j] * y_a[j];
        }
    }
    normalizeQuaternion();

    float I_KH[4][4] = {0};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            float kh = 0.0f;
            for(int k=0; k<3; ++k) {
                kh += Ka[i][k] * Ha[k][j];
            }
            I_KH[i][j] = (i == j) ? 1.0f - kh : -kh;
        }
    }
    float updatedP[4][4] = {0};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            for(int k=0; k<4; ++k) {
                updatedP[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }
    std::memcpy(P, updatedP, sizeof(P));

    // ==========================================
    // 2단계: 지자계 업데이트
    // ==========================================
    float hx = mx*(q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]) + my*2.0f*(q[1]*q[2] - q[0]*q[3]) + mz*2.0f*(q[1]*q[3] + q[0]*q[2]);
    float hy = mx*2.0f*(q[1]*q[2] + q[0]*q[3]) + my*(q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3]) + mz*2.0f*(q[2]*q[3] - q[0]*q[1]);
    
    float bx = std::sqrt(hx*hx + hy*hy);
    float bz = mx*2.0f*(q[1]*q[3] - q[0]*q[2]) + my*2.0f*(q[2]*q[3] + q[0]*q[1]) + mz*(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

    float wx_m = bx*(q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]) + bz*2.0f*(q[1]*q[3] - q[0]*q[2]);
    float wy_m = bx*2.0f*(q[1]*q[2] - q[0]*q[3]) + bz*2.0f*(q[2]*q[3] + q[0]*q[1]);
    float wz_m = bx*2.0f*(q[1]*q[3] + q[0]*q[2]) + bz*(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

    // [오타 완전 박멸] 롤 회전 왜곡을 방지하기 위해 정교하게 부호를 동기화한 야코비안 행렬 Hm
    float Hm[3][4] = {
        { 2.0f*bx*q[0]-2.0f*bz*q[2],  2.0f*bx*q[1]+2.0f*bz*q[3], -2.0f*bx*q[2]-2.0f*bz*q[0], -2.0f*bx*q[3]+2.0f*bz*q[1]},
        {-2.0f*bx*q[3]+2.0f*bz*q[1],  2.0f*bx*q[2]+2.0f*bz*q[0],  2.0f*bx*q[1]+2.0f*bz*q[3], -2.0f*bx*q[0]+2.0f*bz*q[2]},
        { 2.0f*bx*q[2]+2.0f*bz*q[0],  2.0f*bx*q[3]-2.0f*bz*q[1],  2.0f*bx*q[0]-2.0f*bz*q[2],  2.0f*bx*q[1]+2.0f*bz*q[3]}
    };

    float y_m[3] = { mx - wx_m, my - wy_m, mz - wz_m };

    float HmP[3][4] = {0};
    for(int i=0; i<3; ++i) {
        for(int j=0; j<4; ++j) {
            for(int k=0; k<4; ++k) {
                HmP[i][j] += Hm[i][k] * P[k][j];
            }
        }
    }

    float Sm[3][3] = {0};
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            for(int k=0; k<4; ++k) {
                Sm[i][j] += HmP[i][k] * Hm[j][k];
            }
            if (i == j) Sm[i][j] += R_mag;
        }
    }

    float Sm_inv[3][3];
    matrixInversion3x3(Sm, Sm_inv);

    float Km[4][3] = {0};
    float PHmT[4][3] = {0};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<3; ++j) {
            for(int k=0; k<4; ++k) {
                PHmT[i][j] += P[i][k] * Hm[j][k];
            }
        }
    }

    for(int i=0; i<4; ++i) {
        for(int j=0; j<3; ++j) {
            for(int k=0; k<3; ++k) {
                Km[i][j] += PHmT[i][k] * Sm_inv[k][j];
            }
        }
    }

    for(int i=0; i<4; ++i) {
        for(int j=0; j<3; ++j) {
            q[i] += Km[i][j] * y_m[j];
        }
    }
    normalizeQuaternion();

    float finalI_KH[4][4] = {0};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            float kh = 0.0f;
            for(int k=0; k<3; ++k) {
                kh += Km[i][k] * Hm[k][j];
            }
            finalI_KH[i][j] = (i == j) ? 1.0f - kh : -kh;
        }
    }
    float finalP[4][4] = {0};
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            for(int k=0; k<4; ++k) {
                finalP[i][j] += finalI_KH[i][k] * P[k][j];
            }
        }
    }
    std::memcpy(P, finalP, sizeof(P));
}

Attitude_t KalmanFilter::getEuler() const {
    std::lock_guard<std::mutex> lock(mtx);
    Attitude_t euler;

    // 왼쪽 롤 동작 시 화면과 일치하도록 유연하게 반응하는 대수 구속 정렬
    euler.roll = std::atan2(2.0f * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
    
    float sinp = 2.0f * (q[0]*q[2] - q[1]*q[3]);
    if (std::abs(sinp) >= 1.0f)
        euler.pitch = std::copysign(M_PI / 2.0f, sinp);
    else
        euler.pitch = std::asin(sinp);

    // [완전 수정] 인위적인 가공 없이, 시계 방향 회전 시 양수로 자연스럽게 증가하는 정방향 규격 보장
    euler.yaw = std::atan2(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));

    return euler;
}


void KalmanFilter::normalizeQuaternion() {
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);  
    // 수치 해석적 안정성을 위해 0.0f 대신 아주 작은 값(epsilon)으로 검사합니다.
    if (norm > 1e-6f) { 
        q[0] /= norm; 
        q[1] /= norm; 
        q[2] /= norm; 
        q[3] /= norm; 
    } else {
        // [비상 조치] 쿼터니언이 무너졌으므로 수평 상태의 기본 원점(Identity)으로 강제 초기화
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        
        // 오차 공분산 행렬(P)도 초기 불확실성 상태로 리셋하여 필터가 처음부터 다시 학습하도록 유도
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 0.1f : 0.0f; // 오차가 커진 상태로 셋팅
            }
        }
    }
}

void KalmanFilter::matrixInversion3x3(const float in[3][3], float out[3][3]) {
    float det = in[0][0] * (in[1][1] * in[2][2] - in[1][2] * in[2][1]) -
                in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0]) +
                in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);
    if (std::abs(det) < 1e-6f) return;
    float invdet = 1.0f / det;
    out[0][0] = (in[1][1] * in[2][2] - in[1][2] * in[2][1]) * invdet;
    out[0][1] = (in[0][2] * in[2][1] - in[0][1] * in[2][2]) * invdet;
    out[0][2] = (in[0][1] * in[1][2] - in[0][2] * in[1][1]) * invdet;
    out[1][0] = (in[1][2] * in[2][0] - in[1][0] * in[2][2]) * invdet;
    out[1][1] = (in[0][0] * in[2][2] - in[0][2] * in[2][0]) * invdet;
    out[1][2] = (in[0][2] * in[1][0] - in[0][0] * in[1][2]) * invdet;
    out[2][0] = (in[1][0] * in[2][1] - in[1][1] * in[2][0]) * invdet;
    out[2][1] = (in[0][1] * in[2][0] - in[0][0] * in[2][1]) * invdet;
    out[2][2] = (in[0][0] * in[1][1] - in[0][1] * in[1][0]) * invdet;
}

} // namespace Filter
