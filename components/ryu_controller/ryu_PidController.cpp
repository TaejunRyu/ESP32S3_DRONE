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

void PidControl::setAltitudeParams(const AltitudeParams& params) {
    _alt_p = params;
}

void PidControl::reset() {
    for (int i = 0; i < 3; ++i) {
        _angle_integral[i] = 0.0f;
        _angle_prev_err[i] = 0.0f;
        _rate_integral[i]  = 0.0f;
        _rate_prev_err[i]  = 0.0f;
    }
    // 고도 메모리 공간 리셋 동기화
    _alt_vel_integral = 0.0f;
    _alt_vel_prev_err = 0.0f;
}

float PidControl::updateSinglePid(float error, float& integral, float& prev_error, const PidParams& params, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. P 제어량 산출
    float p_out = params.kp * error;

    // 2. I 제어량 산출 (하드웨어 한계를 고려한 Anti-windup 안전 차단 가동)
    integral += error * dt;
    if (integral > params.i_limit)       integral = params.i_limit;
    else if (integral < -params.i_limit) integral = -params.i_limit;
    float i_out = params.ki * integral;

    // 3. D 제어량 산출 (1ms 미세 지터 영향 억제)
    float derivative = (error - prev_error) / dt;
    float d_out = params.kd * derivative;
    prev_error = error;

    // 4. 총합 및 한계선 구속(Saturation)
    float total_out = p_out + i_out + d_out;
    if (total_out > params.out_limit)       total_out = params.out_limit;
    else if (total_out < -params.out_limit) total_out = -params.out_limit;

    return total_out;
}

Vector3f PidControl::updateCascade(const Attitude_t& target_att, 
                                   const Attitude_t& current_att, 
                                   const Vector3f& current_rate, 
                                   float dt) {
    Vector3f control_output = {0.0f, 0.0f, 0.0f};

    // -------------------------------------------------------------
    // 1단계: Outer Loop - 자세 각도 제어 (라디안 기반 오차)
    // -------------------------------------------------------------
    float roll_err  = target_att.roll - current_att.roll;
    float pitch_err = target_att.pitch - current_att.pitch;
    float yaw_err   = target_att.yaw - current_att.yaw;

    // NED 기준 Yaw 각도 점프 현상 완벽 방지 보정선 처리 (-M_PI ~ +M_PI)
    while (yaw_err > M_PI)  yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    // 바깥 루프 출력이 안쪽 루프의 '목표 기체 회전 각속도'로 변환 전파됩니다.
    float target_roll_rate  = updateSinglePid(roll_err,  _angle_integral[0], _angle_prev_err[0], _angle_p_roll,  dt);
    float target_pitch_rate = updateSinglePid(pitch_err, _angle_integral[1], _angle_prev_err[1], _angle_p_pitch, dt);
    float target_yaw_rate   = updateSinglePid(yaw_err,   _angle_integral[2], _angle_prev_err[2], _angle_p_yaw,   dt);

    // -------------------------------------------------------------
    // 2단계: Inner Loop - 각속도 제어 (목표 각속도 - 정제 자이로 속도)
    // -------------------------------------------------------------
    float roll_rate_err  = target_roll_rate - current_rate.x;
    float pitch_rate_err = target_pitch_rate - current_rate.y;
    float yaw_rate_err   = target_yaw_rate - current_rate.z;

    // 액추에이터 믹싱용 최종 축별 가감산 제어 토크 지표 산출
    control_output.x = updateSinglePid(roll_rate_err,  _rate_integral[0], _rate_prev_err[0], _rate_p_roll,  dt);
    control_output.y = updateSinglePid(pitch_rate_err, _rate_integral[1], _rate_prev_err[1], _rate_p_pitch, dt);
    control_output.z = updateSinglePid(yaw_rate_err,   _rate_integral[2], _rate_prev_err[2], _rate_p_yaw,   dt);

    return control_output;
}

float PidControl::updateAltitudeCascade(float target_alt, float current_alt, float current_vel, float dt) {
    if (dt <= 0.0f) return _alt_p.hover_throttle;

    // -------------------------------------------------------------
    // 1단계: Outer Loop - 고도 변위 제어 (m -> m/s 변환)
    // -------------------------------------------------------------
    float alt_error = target_alt - current_alt;
    float target_vel = _alt_p.kp_alt * alt_error;

    // 기체의 과도한 급상승 및 급강하 돌진을 막기 위한 물리 속도 포화 제한
    if (target_vel > _alt_p.vel_limit)  target_vel = _alt_p.vel_limit;
    if (target_vel < -_alt_p.vel_limit) target_vel = -_alt_p.vel_limit;

    // -------------------------------------------------------------
    // 2단계: Inner Loop - 수직 속도 제어 (m/s -> 스로틀 % 출력량 유도)
    // -------------------------------------------------------------
    float vel_error = target_vel - current_vel;

    // P 비례 성분
    float p_out = _alt_p.kp_vel * vel_error;

    // I 적분 성분 (Anti-windup: 기체가 지면이나 장애물에 걸려 뜨지 못할 때 오차가 끝없이 커지는 현상 15% 캡 차단)
    _alt_vel_integral += vel_error * dt;
    if (_alt_vel_integral > 15.0f)  _alt_vel_integral = 15.0f;
    if (_alt_vel_integral < -15.0f) _alt_vel_integral = -15.0f;
    float i_out = _alt_p.ki_vel * _alt_vel_integral;

    // D 미분 성분 (속도 오차의 순간 변화 감지)
    float d_out = _alt_p.kd_vel * (vel_error - _alt_vel_prev_err) / dt;
    _alt_vel_prev_err = vel_error;

    // 💡 [피드포워드 매싱 물리식] 중력 상쇄용 상시 기본 호버 스로틀 수치에 PID 가감산 처리 완료
    float final_throttle = _alt_p.hover_throttle + p_out + i_out + d_out;

    // 모터의 과도한 스핀 및 꺼짐을 방지하기 위한 토크 한계선 고정
    if (final_throttle > _alt_p.out_limit) final_throttle = _alt_p.out_limit;
    if (final_throttle < 10.0f)            final_throttle = 10.0f; // 최저 스로틀 유지 마진선 확보

    return final_throttle;
}

} // namespace Controller




// #include "ryu_PidController.hpp"
// #include "esp_log.h"

// void flight_control_task(void* pvParameters) {
//     // 1. 싱글톤 제어기 획득
//     auto& pid = Controller::PidControl::getInstance();

//     // 2. [자세 게인 튜닝 파라미터 주입] 구조: {Kp, Ki, Kd, I_Limit, Out_Limit}
//     Controller::PidParams att_angle = {2.5f, 0.0f, 0.0f, 0.0f, 10.0f};  // 바깥 각도 루프
//     Controller::PidParams att_rate  = {0.08f, 0.02f, 0.001f, 0.5f, 40.0f}; // 안쪽 각속도 루프
//     pid.setAngleParams(att_angle, att_angle, att_angle);
//     pid.setRateParams(att_rate, att_rate, att_rate);

//     // 3. [고도 게인 튜닝 파라미터 독립 주입]
//     Controller::AltitudeParams alt_config;
//     alt_config.kp_alt         = 1.2f;   // 고도 -> 속도 변환율
//     alt_config.kp_vel         = 1.8f;   // 속도 P
//     alt_config.ki_vel         = 0.4f;   // 속도 I
//     alt_config.kd_vel         = 0.01f;  // 속도 D
//     alt_config.vel_limit      = 2.0f;   // 최대 2m/s
//     alt_config.hover_throttle = 43.5f;  // 이 기체의 공중 유지 호버링 스로틀 추정치 43.5%
//     alt_config.out_limit      = 85.0f;  // 최대 출력 한계 제한
//     pid.setAltitudeParams(alt_config);

//     pid.reset();

//     const float dt = 0.001f; // 1ms 초고속 고정 루프 주기
//     uint32_t loop_cnt = 0;

//     // 가상의 목표치 선언 (호버링 벤치마킹 타겟)
//     Attitude_t target_pose = {0.0f, 0.0f, 0.0f}; // 수평 유지
//     float target_altitude  = 1.5f;               // 1.5m 고도 홀딩 명령

//     while (true) {
//         vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));

//         // [이전 단계들 연동 소스 매핑]
//         // 1) EKF 가 계산한 현재 자세 각도 및 수직 가속도 융합 고도/속도 수렴 데이터 취득
//         Attitude_t current_pose = Filter::KalmanFilter::getInstance().getEuler();
//         Vector3f current_gyro   = get_clean_gyro_rad_per_sec();
        
//         float current_alt = Filter::VerticalFilter::getInstance().getAltitude();
//         float current_vel = Filter::VerticalFilter::getInstance().getVelocity();

//         // -------------------------------------------------------------
//         // 핵심 연산: 자세와 고도 제어 명령을 병렬 독립 연산 처리
//         // -------------------------------------------------------------
//         Vector3f att_outputs = pid.updateCascade(target_pose, current_pose, current_gyro, dt);
//         float base_throttle  = pid.updateAltitudeCascade(target_altitude, current_alt, current_vel, dt);

//         // -------------------------------------------------------------
//         // 최종 믹싱 단계: 고도 스로틀(Base)에 자세 복원력을 축별 가감산 (Quadcopter X-Type 기준)
//         // -------------------------------------------------------------
//         float motor1_FR = base_throttle - att_outputs.x - att_outputs.y - att_outputs.z; // 전방 우측
//         float motor2_BL = base_throttle + att_outputs.x + att_outputs.y - att_outputs.z; // 후방 좌측
//         float motor3_FL = base_throttle + att_outputs.x - att_outputs.y + att_outputs.z; // 전방 좌측
//         float motor4_BR = base_throttle - att_outputs.x + att_outputs.y + att_outputs.z; // 후방 우측

//         // 하드웨어 하위 모터 드라이버(MCPWM/ESC 10%~90%) 입력 마진 가이드 인가부 연동 생략...

//         // 축약 로깅 처리 출력 부
//         if (++loop_cnt >= 20) { 
//             loop_cnt = 0;
//             ESP_LOGI("FLIGHT", "Th: %5.1f%% | RollOut: %5.2f | PitchOut: %5.2f | YawOut: %5.2f", 
//                      base_throttle, att_outputs.x, att_outputs.y, att_outputs.z);
//         }
//     }
// }
