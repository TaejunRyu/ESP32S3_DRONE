#pragma once

#include "ryu_Types.hpp" // Vector3f, Attitude_t, BaroData 등 프로젝트 공용 구조체 정의 가정

namespace Controller {

// 자세(Roll, Pitch, Yaw) 제어용 일반 PID 코어 구조체
struct PidParams {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float i_limit = 0.0f;
    float out_limit = 0.0f;
};

// 고도(Altitude) 제어용 특화 캐스케이드 파라미터 구조체 (독립 분리)
struct AltitudeParams {
    float kp_alt = 1.0f;          // 바깥쪽 루프 (고도 오차 m -> 목표 속도 m/s 변환 이득)
    float kp_vel = 2.0f;          // 안쪽 속도 루프 P 이득
    float ki_vel = 0.5f;          // 안쪽 속도 루프 I 이득 (기체 처짐 및 무게 중심 상쇄 핵심)
    float kd_vel = 0.02f;         // 안쪽 속도 루프 D 이득 (고주파 출렁임 및 떨림 억제)
    float vel_limit = 2.5f;       // 안전을 위한 최대 상승/하강 속도 제한 (m/s)
    float hover_throttle = 45.0f; // 호버링 유지를 위한 기본 스로틀 베이스 피드포워드 파워 (%)
    float out_limit = 85.0f;      // 최대 스로틀 출력 마진 제한 (%)
};

class PidControl {
public:
    // 스레드 세이프가 보장되는 로컬 정적 싱글톤 인스턴스 반환 (Magic Statics)
    static PidControl& getInstance() {
        static PidControl instance;
        return instance;
    }

    // 싱글톤 구조 보장을 위한 복사 및 대입/이동 원천 차단
    PidControl(const PidControl&) = delete;
    PidControl& operator=(const PidControl&) = delete;
    PidControl(PidControl&&) = delete;
    PidControl& operator=(PidControl&&) = delete;

    // 자세 파라미터 설정 함수
    void setAngleParams(const PidParams& roll, const PidParams& pitch, const PidParams& yaw);
    void setRateParams(const PidParams& roll, const PidParams& pitch, const PidParams& yaw);

    // 고도 파라미터 설정 함수 (독립 설정 분리 완료)
    void setAltitudeParams(const AltitudeParams& params);

    /**
     * @brief 자세(Roll, Pitch, Yaw) 캐스케이드 2중 루프 PID 연산 (1000Hz 스케줄링)
     * @param target_att 목표 각도 자세 (라디안 단위)
     * @param current_att EKF 필터가 정제한 현재 각도 자세 (라디안 단위)
     * @param current_rate 자이로 센서 정전 처리된 각 축별 현재 각속도 (rad/s 단위, x=roll, y=pitch, z=yaw)
     * @param dt 루프 주기 (0.001f)
     * @return Vector3f 모터 믹싱에 가감산할 최종 자세 제어 출력값 (x, y, z)
     */
    Vector3f updateCascade(const Attitude_t& target_att, 
                           const Attitude_t& current_att, 
                           const Vector3f& current_rate, 
                           float dt);

    /**
     * @brief 고도(Altitude) 캐스케이드 2중 루프 PID 연산 (1000Hz 스케줄링)
     * @param target_alt 목표 고도 (m)
     * @param current_alt 수직 칼만 필터가 추정한 현재 정밀 고도 (m)
     * @param current_vel 수직 칼만 필터가 추정한 현재 정밀 상승/하강 속도 (m/s)
     * @param dt 루프 주기 (0.001f)
     * @return float 4개 모터의 베이스 추력으로 그대로 주입할 최종 Throttle 양수 백분율 (% 값)
     */
    float updateAltitudeCascade(float target_alt, float current_alt, float current_vel, float dt);

    // 내부 누적 적분 메모리 및 이전 오차 초기화 (이착륙, 모드 긴급 스위칭 시 필수 호출)
    void reset();

private:
    PidControl() = default;
    ~PidControl() = default;

    // 수동 오버헤드를 줄인 단일 내부 PID 축 제어용 유틸리티 메서드
    float updateSinglePid(float error, float& integral, float& prev_error, const PidParams& params, float dt);

    // 자세 및 고도 PID 파라미터 저장소 명시적 격리
    PidParams _angle_p_roll, _angle_p_pitch, _angle_p_yaw;
    PidParams _rate_p_roll,  _rate_p_pitch,  _rate_p_yaw;
    AltitudeParams _alt_p;

    // 자세 제어용 독립 내부 적분 및 미분 메모리 공간
    float _angle_integral[3] = {0.0f, 0.0f, 0.0f}; // 0:Roll, 1:Pitch, 2:Yaw
    float _angle_prev_err[3] = {0.0f, 0.0f, 0.0f};
    
    float _rate_integral[3]  = {0.0f, 0.0f, 0.0f};
    float _rate_prev_err[3]  = {0.0f, 0.0f, 0.0f};

    // 고도 제어용 독립 내부 적분 및 미분 메모리 공간 (자세 데이터 오염 원천 분리)
    float _alt_vel_integral = 0.0f;
    float _alt_vel_prev_err = 0.0f;
};

} // namespace Controller
