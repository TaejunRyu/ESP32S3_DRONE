#pragma once

#include <cmath>
#include <mutex>
#include "ryu_Types.hpp"

namespace Filter {

class KalmanFilter {
public:
    // 싱글톤 인스턴스 획득 (Thread-Safe)
    static KalmanFilter& getInstance() {
        static KalmanFilter instance;
        return instance;
    }

    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;

    // 필터 노이즈 매개변수 초기화
    void init(float q_gyro, float q_angle, float r_accel, float r_mag);
    
    // acc: g 단위, gyro_rad: 태스크단에서 변환된 rad/s 단위, mag: 정규화 데이터
    void update(const Vector3f& acc, const Vector3f& gyro_rad, const Vector3f& mag, float dt);
    
    // 내부 추정 라디안 값을 그대로 반환하는 오리지널 함수
    Attitude_t getEuler() const;

private:
    KalmanFilter();
    ~KalmanFilter() = default;

    mutable std::mutex mtx_;

    Vector3f estimated_attitude_; // x: Roll, y: Pitch, z: Yaw (Radian)
    Vector3f bias_;               // 자이로 바이어스 추정치 (Radian/s)

    // 각 축별 독립 2x2 오차 공분산 행렬 요소 (메모리 버그 방지를 위해 개별 스칼라 선언)
    float Px00, Px01, Px10, Px11;
    float Py00, Py01, Py10, Py11;
    float Pz00, Pz01, Pz10, Pz11;

    // 칼만 필터 가중치 노이즈 변수
    float Q_angle_;
    float Q_gyro_;
    float R_accel_;
    float R_mag_;

    void normalizeVector(Vector3f& v);
};

} // namespace Filter
