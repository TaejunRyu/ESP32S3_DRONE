#include "ryu_PidController.hpp"
#include <cmath>

namespace Controller {

void PidControl::setAngleParams(const PidParams& roll, const PidParams& pitch, const PidParams& yaw) {
    _angle_p_roll  = roll;
    _angle_p_pitch = pitch;
    _angle_p_yaw   = yaw;
}

void PidControl::setRateParams(const PidParams& roll, const PidParams& pitch, const PidParams& yaw) {
    _rate_p_roll  = roll;
    _rate_p_pitch = pitch;
    _rate_p_yaw   = yaw;
}

void PidControl::reset() {
    _state_angle_roll  = {0.0f, 0.0f};
    _state_angle_pitch = {0.0f, 0.0f};
    _state_angle_yaw   = {0.0f, 0.0f};
    
    _state_rate_roll   = {0.0f, 0.0f};
    _state_rate_pitch  = {0.0f, 0.0f};
    _state_rate_yaw    = {0.0f, 0.0f};
}

float PidControl::updateSinglePid(float error, PidState& state, const PidParams& params, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // P 제어
    float p_out = params.kp * error;

    // I 제어 (Anti-windup 적분 제한 적용)
    state.integral += error * dt;
    if (state.integral > params.i_limit) state.integral = params.i_limit;
    else if (state.integral < -params.i_limit) state.integral = -params.i_limit;
    float i_out = params.ki * state.integral;

    // D 제어
    float derivative = (error - state.prev_error) / dt;
    float d_out = params.kd * derivative;
    state.prev_error = error;

    // 최종 출력 제한
    float total_out = p_out + i_out + d_out;
    if (total_out > params.out_limit) total_out = params.out_limit;
    else if (total_out < -params.out_limit) total_out = -params.out_limit;

    return total_out;
}

Vector3f PidControl::updateCascade(const Attitude_t& target_att, 
                                   const Attitude_t& current_att, 
                                   const Vector3f& current_rate, 
                                   float dt) {
    Vector3f control_output = {0.0f, 0.0f, 0.0f};

    // --- 1단계: Outer Loop (자세 각도 제어) ---
    float roll_err  = target_att.roll - current_att.roll;
    float pitch_err = target_att.pitch - current_att.pitch;
    float yaw_err   = target_att.yaw - current_att.yaw;

    // NED 기준 Yaw 주기성 보정 (-PI ~ +PI)
    while (yaw_err > M_PI)  yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    // 각 축별 완벽히 분리된 구조체 전달
    float target_roll_rate  = updateSinglePid(roll_err,  _state_angle_roll,  _angle_p_roll,  dt);
    float target_pitch_rate = updateSinglePid(pitch_err, _state_angle_pitch, _angle_p_pitch, dt);
    float target_yaw_rate   = updateSinglePid(yaw_err,   _state_angle_yaw,   _angle_p_yaw,   dt);

    // --- 2단계: Inner Loop (각속도 제어) ---
    float roll_rate_err  = target_roll_rate - current_rate.x;
    float pitch_rate_err = target_pitch_rate - current_rate.y;
    float yaw_rate_err   = target_yaw_rate - current_rate.z;

    // 최종 제어 액추에이터 제어량 산출
    control_output.x = updateSinglePid(roll_rate_err,  _state_rate_roll,  _rate_p_roll,  dt);
    control_output.y = updateSinglePid(pitch_rate_err, _state_rate_pitch, _rate_p_pitch, dt);
    control_output.z = updateSinglePid(yaw_rate_err,   _state_rate_yaw,   _rate_p_yaw,   dt);

    return control_output;
}

} // namespace Controller
