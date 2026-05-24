#pragma once

#include "ryu_Types.hpp"

namespace Controller {

struct PidParams {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float i_limit = 0.0f;
    float out_limit = 0.0f;
};

// 내부 루프 오차 관리를 위한 독립 구조체 정의 (배열 버그 원천 차단)
struct PidState {
    float integral = 0.0f;
    float prev_error = 0.0f;
};

class PidControl {
public:
    static PidControl& getInstance() {
        static PidControl instance;
        return instance;
    }

    PidControl(const PidControl&) = delete;
    PidControl& operator=(const PidControl&) = delete;

    void setAngleParams(const PidParams& roll, const PidParams& pitch, const PidParams& yaw);
    void setRateParams(const PidParams& roll, const PidParams& pitch, const PidParams& yaw);

    Vector3f updateCascade(const Attitude_t& target_att, 
                           const Attitude_t& current_att, 
                           const Vector3f& current_rate, 
                           float dt);

    void reset();

private:
    PidControl() = default;
    ~PidControl() = default;

    // 단일 PID 연산 유틸리티
    float updateSinglePid(float error, PidState& state, const PidParams& params, float dt);

    // 파라미터 명시적 분리
    PidParams _angle_p_roll,  _angle_p_pitch,  _angle_p_yaw;
    PidParams _rate_p_roll,   _rate_p_pitch,   _rate_p_yaw;

    // 내부 상태 상태 변수 명시적 분리 (메모리 침범 가능성 0%)
    PidState _state_angle_roll,  _state_angle_pitch,  _state_angle_yaw;
    PidState _state_rate_roll,   _state_rate_pitch,   _state_rate_yaw;
};

} // namespace Controller
