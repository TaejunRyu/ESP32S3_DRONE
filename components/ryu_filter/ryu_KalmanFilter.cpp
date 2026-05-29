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
#include "esp_dsp.h" // esp-dsp 통합 헤더 추가

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

    // 1. 쿼터니언 미분 방정식 누적 적분 (기존 검증된 오리지널 수식 구조 유지)
    float dq0 = 0.5f * (-q[1]*wx - q[2]*wy - q[3]*wz) * dt;
    float dq1 = 0.5f * ( q[0]*wx - q[3]*wy + q[2]*wz) * dt;
    float dq2 = 0.5f * ( q[3]*wx + q[0]*wy - q[1]*wz) * dt;
    float dq3 = 0.5f * (-q[2]*wx + q[1]*wy + q[0]*wz) * dt;

    q[0] += dq0; q[1] += dq1; q[2] += dq2; q[3] += dq3;
    normalizeQuaternion();

    // 시스템 오차 전파 야코비안 행렬 F (4x4)
    float F[4][4] = {
        {1.0f,       -0.5f*wx*dt, -0.5f*wy*dt, -0.5f*wz*dt},
        {0.5f*wx*dt,  1.0f,        0.5f*wz*dt, -0.5f*wy*dt},
        {0.5f*wy*dt, -0.5f*wz*dt,  1.0f,        0.5f*wx*dt},
        {0.5f*wz*dt,  0.5f*wy*dt, -0.5f*wx*dt,  1.0f}
    };

    // 💡 [가속화 1] nextP = F(4x4) * P(4x4) = nextP(4x4) 
    // 수동 3중 for 루프를 단 한 줄의 하드웨어 PIE 확장 명령어로 압축 처리합니다.
    float nextP[4][4] = {0};
    dspm_mult_f32((float*)F, (float*)P, (float*)nextP, 4, 4, 4);

    // 💡 [가속화 2] P = nextP(4x4) * F^T(4x4) + Q_gyro * dt
    // esp-dsp 하드웨어 가속기가 오작동 없이 처리할 수 있도록 F^T 전치 행렬 버퍼를 평탄화 복사합니다.
    float F_T[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            F_T[j][i] = F[i][j];
        }
    }

    // nextP(4x4) * F_T(4x4) 연산을 하드웨어 벡터 프로세서로 초고속 수행
    float updatedP[4][4] = {0};
    dspm_mult_f32((float*)nextP, (float*)F_T, (float*)updatedP, 4, 4, 4);

    // 대각 성분에 프로세스 노이즈 공분산(Q_gyro * dt) 병합 및 최종 복사
    float q_noise = Q_gyro * dt;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = (i == j) ? updatedP[i][j] + q_noise : updatedP[i][j];
        }
    }
}



void KalmanFilter::update(const Vector3f& accel, const Vector3f& mag) {
    std::lock_guard<std::mutex> lock(mtx);

    // ==========================================
    // 0단계: 각각 독립적인 유효성 검사 수행 (esp-dsp 가속)
    // ==========================================
    float a_sq_sum = accel.x*accel.x + accel.y*accel.y + accel.z*accel.z;
    float a_norm = 0.0f;
    dsps_sqrt_f32(&a_sq_sum, &a_norm, 1); 
    bool is_accel_valid = (a_norm > 0.001f);

    // ==========================================
    // 1단계: 가속도계 업데이트
    // ==========================================
    if (is_accel_valid) {
        float ax = accel.x / a_norm; 
        float ay = accel.y / a_norm; 
        float az = accel.z / a_norm;

        float vx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
        float vy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
        float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

        float Ha[3][4] = {
            {-2.0f*q[2],  2.0f*q[3], -2.0f*q[0],  2.0f*q[1]},
            { 2.0f*q[1],  2.0f*0.0f,  2.0f*q[3],  2.0f*q[2]}, // 기존 정석 매핑 유지
            { 2.0f*q[0], -2.0f*q[1], -2.0f*q[2],  2.0f*q[3]}
        };
        // ※ 주의: 기존 원본의 Ha[1][1]이 2.0f*q[0]에서 2.0f*0.0f(또는 오타) 구조였다면 원본을 존중합니다.
        Ha[1][1] = 2.0f * q[0]; // 정석 매핑 복원 필요시 참고

        float y_a[3] = { ax - vx, ay - vy, az - vz };
        
        // 💡 [가속화 1] Ha(3x4) * P(4x4) = HaP(3x4) 
        float HaP[3][4] = {0};
        dspm_mult_f32((float*)Ha, (float*)P, (float*)HaP, 3, 4, 4);

        // 💡 [가속화 2] Sa = HaP * Ha^T + R_accel (메모리 정방향 전치 배치 후 dspm 연산)
        float Ha_T[4][3] = {0};
        for(int i=0; i<3; ++i) {
            for(int j=0; j<4; ++j) {
                Ha_T[j][i] = Ha[i][j];
            }
        }
        float Sa[3][3] = {0};
        dspm_mult_f32((float*)HaP, (float*)Ha_T, (float*)Sa, 3, 4, 3);
        Sa[0][0] += R_accel; Sa[1][1] += R_accel; Sa[2][2] += R_accel;

        float Sa_inv[3][3];
        matrixInversion3x3(Sa, Sa_inv);

        // 💡 [가속화 3] PHaT = P(4x4) * Ha_T(4x3) = PHaT(4x3)
        float PHaT[4][3] = {0};
        dspm_mult_f32((float*)P, (float*)Ha_T, (float*)PHaT, 4, 4, 3);

        // 💡 [가속화 4] Ka = PHaT(4x3) * Sa_inv(3x3) = Ka(4x3)
        float Ka[4][3] = {0};
        dspm_mult_f32((float*)PHaT, (float*)Sa_inv, (float*)Ka, 4, 3, 3);

        // 💡 [가속화 5] q 업데이트 벡터곱 최적화: Ka(4x3) * y_a(3x1) = dq_update(4x1)
        float dq_update[4] = {0.0f};
        dspm_mult_f32((float*)Ka, (float*)y_a, dq_update, 4, 3, 1);
        q[0] += dq_update[0]; q[1] += dq_update[1]; q[2] += dq_update[2]; q[3] += dq_update[3];
        normalizeQuaternion();

        // 💡 [가속화 6] I_KH 구축 가속화: Ka(4x3) * Ha(3x4) = KaHa(4x4)
        float KaHa[4][4] = {0};
        dspm_mult_f32((float*)Ka, (float*)Ha, (float*)KaHa, 4, 3, 4);
        
        float I_KH[4][4] = {0};
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                I_KH[i][j] = (i == j) ? 1.0f - KaHa[i][j] : -KaHa[i][j];
            }
        }

        // 💡 [가속화 7] updatedP = I_KH(4x4) * P(4x4) = updatedP(4x4)
        float updatedP[4][4] = {0};
        dspm_mult_f32((float*)I_KH, (float*)P, (float*)updatedP, 4, 4, 4);
        std::memcpy(P, updatedP, sizeof(P));
    }

    // ==========================================
    // 2단계: 지자계 업데이트
    // ==========================================
    float m_sq_sum = mag.x*mag.x + mag.y*mag.y + mag.z*mag.z;
    float m_norm = 0.0f;
    dsps_sqrt_f32(&m_sq_sum, &m_norm, 1); 

    bool is_mag_valid = (m_norm > 0.001f);
        
    if (is_mag_valid) {    
        
        float mx = mag.x / m_norm; 
        float my = -mag.y / m_norm; 
        float mz = mag.z / m_norm; 

        float hx = mx*(q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]) + my*2.0f*(q[1]*q[2] - q[0]*q[3]) + mz*2.0f*(q[1]*q[3] + q[0]*q[2]);
        float hy = mx*2.0f*(q[1]*q[2] + q[0]*q[3]) + my*(q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3]) + mz*2.0f*(q[2]*q[3] - q[0]*q[1]);
        
        // 💡 필요 시 이 부하가 큰 수동 std::sqrt도 dsps_sqrt_f32 구문 가속 전환 가능
        float h_sq_sum = hx*hx + hy*hy;
        float bx = 0.0f;
        dsps_sqrt_f32(&h_sq_sum, &bx, 1);
        
        float bz = mx*2.0f*(q[1]*q[3] - q[0]*q[2]) + my*2.0f*(q[2]*q[3] + q[0]*q[1]) + mz*(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

        float wx_m = bx*(q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]) + bz*2.0f*(q[1]*q[3] - q[0]*q[2]);
        float wy_m = bx*2.0f*(q[1]*q[2] - q[0]*q[3]) + bz*2.0f*(q[2]*q[3] + q[0]*q[1]);
        float wz_m = bx*2.0f*(q[1]*q[3] + q[0]*q[2]) + bz*(q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

        float Hm[3][4] = {
            { 2.0f*bx*q[0]-2.0f*bz*q[2],  2.0f*bx*q[1]+2.0f*bz*q[3], -2.0f*bx*q[2]-2.0f*bz*q[0], -2.0f*bx*q[3]+2.0f*bz*q[1]},
            {-2.0f*bx*q[3]+2.0f*bz*q[1],  2.0f*bx*q[2]+2.0f*bz*q[0],  2.0f*bx*q[1]+2.0f*bz*q[3], -2.0f*bx*q[0]+2.0f*bz*q[2]},
            { 2.0f*bx*q[2]+2.0f*bz*q[0],  2.0f*bx*q[3]-2.0f*bz*q[1],  2.0f*bx*q[0]-2.0f*bz*q[2],  2.0f*bx*q[1]+2.0f*bz*q[3]}
        };

        float y_m[3] = { mx - wx_m, my - wy_m, mz - wz_m };

        // 💡 [가속화 8] Hm(3x4) * P(4x4) = HmP(3x4)
        float HmP[3][4] = {0};
        dspm_mult_f32((float*)Hm, (float*)P, (float*)HmP, 3, 4, 4);

        // 💡 [가속화 9] Sm = HmP * Hm^T + R_mag
        float Hm_T[4][3] = {0};
        for(int i=0; i<3; ++i) {
            for(int j=0; j<4; ++j) {
                Hm_T[j][i] = Hm[i][j];
            }
        }
        float Sm[3][3] = {0};
        dspm_mult_f32((float*)HmP, (float*)Hm_T, (float*)Sm, 3, 4, 3);
        Sm[0][0] += R_mag; Sm[1][1] += R_mag; Sm[2][2] += R_mag;

        float Sm_inv[3][3];
        matrixInversion3x3(Sm, Sm_inv);

        // 💡 [가속화 10] PHmT = P(4x4) * Hm_T(4x3) = PHmT(4x3)
        float PHmT[4][3] = {0};
        dspm_mult_f32((float*)P, (float*)Hm_T, (float*)PHmT, 4, 4, 3);

        // 💡 [가속화 11] Km = PHmT(4x3) * Sm_inv(3x3) = Km(4x3)
        float Km[4][3] = {0};
        dspm_mult_f32((float*)PHmT, (float*)Sm_inv, (float*)Km, 4, 3, 3);

        // 💡 [가속화 12] q 업데이트 가속화: Km(4x3) * y_m(3x1) = dq_mag_update(4x1)
        float dq_mag_update[4] = {0.0f};
        dspm_mult_f32((float*)Km, (float*)y_m, dq_mag_update, 4, 3, 1);
        q[0] += dq_mag_update[0]; q[1] += dq_mag_update[1]; q[2] += dq_mag_update[2]; q[3] += dq_mag_update[3];
        normalizeQuaternion();

        // 💡 [가속화 13] KmHm = Km(4x3) * Hm(3x4) = KmHm(4x4)
        float KmHm[4][4] = {0};
        dspm_mult_f32((float*)Km, (float*)Hm, (float*)KmHm, 4, 3, 4);

        float finalI_KH[4][4] = {0};
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                finalI_KH[i][j] = (i == j) ? 1.0f - KmHm[i][j] : -KmHm[i][j];
            }
        }
        
        // 💡 [가속화 14] finalP = finalI_KH(4x4) * P(4x4) = finalP(4x4)
        float finalP[4][4] = {0};
        dspm_mult_f32((float*)finalI_KH, (float*)P, (float*)finalP, 4, 4, 4);
        std::memcpy(P, finalP, sizeof(P));
    }
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
    // 💡 [원본 복원] esp-dsp에 3x3 역행렬 함수가 없으므로 정석 수동 코드를 그대로 사용해야 합니다.
    float det = in[0][0] * (in[1][1] * in[2][2] - in[1][2] * in[2][1]) -
                in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0]) +
                in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);
                
    if (std::abs(det) < 1e-6f) {
        // 역행렬 계산이 불가능한 특이행렬일 때 NaN 발산을 막기 위한 예외 처리 (단위행렬 리셋)
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                out[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return;
    }
    
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

void KalmanFilter::getQuaternion(float out_q[4]) const {
    std::lock_guard<std::mutex> lock(mtx);
    out_q[0] = q[0];
    out_q[1] = q[1];
    out_q[2] = q[2];
    out_q[3] = q[3];
}


float KalmanFilter::get_pure_vertical_accel(Vector3f acc) 
{
    // 1. [핵심] 기존 자세 EKF로부터 실시간 최신 쿼터니언 상태 변수 취득
    // (이 값이 실시간 기체의 롤, 피치 기울임 정보를 온전히 담고 있습니다.)

    
    // 센서 원시 데이터(m/s^2)를 9.81로 나누어 단위를 G 규격(정지 시 1.0)으로 가공
    acc = acc / 9.80665f; // 전체 벡터를 한 번에 G 단위로 변환 (각 축별로 나누는 대신)

    // 3. 취득한 기존 EKF 쿼터니언을 이용하여 체프 가속도를 지구 수직 방향(Z축 Down)으로 회전 투영
    // (q[0], q[1], q[2], q[3]) 쿼터니언이 표현하는 회전 변환을 이용하여 가속도 벡터를 지구 기준으로 변환합니다.
    float acc_z_earth = 2.0f * (q[1]*q[3] - q[0]*q[2]) * acc.x + 
                        2.0f * (q[0]*q[1] + q[2]*q[3]) * acc.y + 
                        (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]) * acc.z;

    // 4. 중력 성분을 제거한 순수 수직 가속도를 m/s^2 단위로 변환하여 필터 예측 단계에 공급
    // (호버링 중에는 0.0f 근처에 머물러야 합니다.)
    // (acc_z_earth가 1.0f보다 크면 상승 가속, 1.0f보다 작으면 하강 가속을 나타냅니다.)
    return (acc_z_earth - 1.0f) * 9.80665f;
}


} // namespace Filter
