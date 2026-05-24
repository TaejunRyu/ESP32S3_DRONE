#include "ryu_VerticalFilter.hpp"
#include <cmath>

namespace Filter {

void VerticalFilter::reset() {
    _alt = 0.0f;
    _vel = 0.0f;
    _acc_bias = 0.0f;
    
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            _P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void VerticalFilter::predict(float acc_z_earth, float dt) {
    // 1. 상태 공간 예측 (물리 등가속도 운동 법칙 적용)
    // acc_pure = 측정된 가속도 - 추정된 가속도 바이어스
    float acc_pure = acc_z_earth - _acc_bias;
    
    _alt += (_vel * dt) + (0.5f * acc_pure * dt * dt);
    _vel += acc_pure * dt;

    // 2. 오차 공분산 행렬 업데이트 (P = F*P*F' + Q) 선형 전개 최적화
    float p00 = _P[0][0] + dt * (_P[1][0] + _P[0][1] + dt * _P[1][1]) - 0.5f * dt * dt * (_P[2][0] + _P[0][2] + dt * _P[2][1] + dt * _P[1][2]);
    float p01 = _P[0][1] + dt * _P[1][1] - 0.5f * dt * dt * _P[2][1] - dt * (_P[0][2] + dt * _P[1][2]);
    float p02 = _P[0][2] + dt * _P[1][2] - 0.5f * dt * dt * _P[2][2];
    
    float p10 = _P[1][0] + dt * _P[1][1] - dt * _P[1][2] - 0.5f * dt * dt * _P[2][2];
    float p11 = _P[1][1] - dt * (_P[1][2] + _P[2][1]) + dt * dt * _P[2][2];
    float p12 = _P[1][2] - dt * _P[2][2];
    
    float p20 = _P[2][0] + dt * _P[2][1] - 0.5f * dt * dt * _P[2][2];
    float p21 = _P[2][1] - dt * _P[2][2];
    float p22 = _P[2][2];

    _P[0][0] = p00 + _Q_alt * dt;  _P[0][1] = p01;                _P[0][2] = p02;
    _P[1][0] = p10;                _P[1][1] = p11 + _Q_vel * dt;  _P[1][2] = p12;
    _P[2][0] = p20;                _P[2][1] = p21;                _P[2][2] = p22 + _Q_bias * dt;
}

void VerticalFilter::update(float baro_alt) {
    // 측정 오차(Innovation) 계산
    float innovation = baro_alt - _alt;
    
    // 칼만 이득(Kalman Gain) 계산 구문 분리
    float S = _P[0][0] + _R_baro;
    if (std::abs(S) < 1e-6f) return;
    float S_inv = 1.0f / S;

    float K[3];
    K[0] = _P[0][0] * S_inv;
    K[1] = _P[1][0] * S_inv;
    K[2] = _P[2][0] * S_inv;

    // 상태 변수 최종 보정
    _alt      += K[0] * innovation;
    _vel      += K[1] * innovation;
    _acc_bias += K[2] * innovation;

    // 오차 공분산 최종 보정 (P = (I - K*H)*P)
    float p00 = _P[0][0] * (1.0f - K[0]); float p01 = _P[0][1] * (1.0f - K[0]); float p02 = _P[0][2] * (1.0f - K[0]);
    float p10 = _P[1][0] - K[1]*_P[0][0]; float p11 = _P[1][1] - K[1]*_P[0][1]; float p12 = _P[1][2] - K[1]*_P[0][2];
    float p20 = _P[2][0] - K[2]*_P[0][0]; float p21 = _P[2][1] - K[2]*_P[0][1]; float p22 = _P[2][2] - K[2]*_P[0][2];

    _P[0][0] = p00; _P[0][1] = p01; _P[0][2] = p02;
    _P[1][0] = p10; _P[1][1] = p11; _P[1][2] = p12;
    _P[2][0] = p20; _P[2][1] = p21; _P[2][2] = p22;
}



// 💡 [신설] GPS 고도 데이터가 도달했을 때만 띄엄띄엄(예: 100ms 마다) 호출되는 보정 함수
void VerticalFilter::updateGPS(float gps_alt) {
    // 측정 오차 계산 (GPS 고도 - 현재 필터 추정 고도)
    float innovation = gps_alt - _alt;
    
    // GPS 전용 칼만 이득(Kalman Gain) 계산 (H 행렬 성분이 고도축이므로 1단계와 대수 전개 동일)
    float S = _P[0][0] + _R_gps;
    if (std::abs(S) < 1e-6f) return;
    float S_inv = 1.0f / S;

    float K[3];
    K[0] = _P[0][0] * S_inv;
    K[1] = _P[1][0] * S_inv;
    K[2] = _P[2][0] * S_inv;

    // GPS 오차를 기반으로 상태 변수(고도, 속도, 가속도 바이어스) 보정
    _alt      += K[0] * innovation;
    _vel      += K[1] * innovation;
    _acc_bias += K[2] * innovation; // 💡 가속도 센서의 영점 드리프트까지 GPS가 잡아줍니다.

    // 오차 공분산 행렬 보정 (P = (I - K*H)*P)
    float p00 = _P[0][0] * (1.0f - K[0]); float p01 = _P[0][1] * (1.0f - K[0]); float p02 = _P[0][2] * (1.0f - K[0]);
    float p10 = _P[1][0] - K[1]*_P[0][0]; float p11 = _P[1][1] - K[1]*_P[0][1]; float p12 = _P[1][2] - K[1]*_P[0][2];
    float p20 = _P[2][0] - K[2]*_P[0][0]; float p21 = _P[2][1] - K[2]*_P[0][1]; float p22 = _P[2][2] - K[2]*_P[0][2];

    _P[0][0] = p00; _P[0][1] = p01; _P[0][2] = p02;
    _P[1][0] = p10; _P[1][1] = p11; _P[1][2] = p12;
    _P[2][0] = p20; _P[2][1] = p21; _P[2][2] = p22;
}

} // namespace Filter
