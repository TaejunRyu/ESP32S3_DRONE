#pragma once

#include "ryu_Types.hpp"

namespace Filter {

class VerticalFilter {
public:
    static VerticalFilter& getInstance() {
        static VerticalFilter instance;
        return instance;
    }

    VerticalFilter(const VerticalFilter&) = delete;
    VerticalFilter& operator=(const VerticalFilter&) = delete;

    /**
     * @brief 1ms 주기 예측 단계 (가속도 기반 고도/속도 적분)
     * @param acc_z_earth 지구 고정 좌표계로 변환된 순수 수직 가속도 (m/s^2, 중력 9.81이 제거된 상태)
     * @param dt 주기 (0.001f)
     */
    void predict(float acc_z_earth, float dt);

    /**
     * @brief 40ms 주기 보정 단계 (기압계 데이터가 들어왔을 때만 호출)
     * @param baro_alt 기압계가 계산한 현재 고도 (m)
     */
    void update(float baro_alt);
    void updateGPS(float gps_alt);

    // 필터링된 최신 수직 상태 반환 함수
    float getAltitude() const { return _alt; }
    float getVelocity() const { return _vel; }

    void reset();

private:
    VerticalFilter() { reset(); }
    ~VerticalFilter() = default;

    // 상태 변수: [0] 고도(m), [1] 속도(m/s), [2] 가속도 바이어스(오프셋 drift)
    float _alt = 0.0f;
    float _vel = 0.0f;
    float _acc_bias = 0.0f;

    // 오차 공분산 행렬 (3x3 대각 성분 초기화)
    float _P[3][3] = {
        {2.0f, 0.0f, 0.0f},
        {0.0f, 2.0f, 0.0f},
        {0.0f, 0.0f, 0.2f}
    };

    // 필터 튜닝 파라미터 (프로세스 노이즈 Q, 측정 노이즈 R)
    const float _Q_alt = 0.005f;
    const float _Q_vel = 0.01f;
    const float _Q_bias = 0.001f;
    const float _R_baro = 0.25f; // 기압계 노이즈 세기 (값이 클수록 기압계를 부드럽게 추종)
    const float _R_gps  = 4.00f; // 💡 GPS 고도는 노이즈가 심하므로 크게 잡아서 천천히 신뢰하도록 설정!
};

} // namespace Filter
