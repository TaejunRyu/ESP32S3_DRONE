#pragma once

#include <cstdint>

// 1. 센서 3축 데이터 (가속도, 자이로 공용)
struct Vector3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3f() = default;
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    // 연산자 오버로딩: 센서 오프셋이나 필터 연산을 직관적으로 만들기 위함
    Vector3f operator+(const Vector3f& other) const {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }
    Vector3f operator-(const Vector3f& other) const {
        return Vector3f(x - other.x, y - other.y, z - other.z);
    }
    Vector3f operator*(float scalar) const {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }
};

// 2. 오일러 각도 기반 드론 자세 데이터
struct Attitude {
    float roll  = 0.0f; // X축 회전 (도 또는 라디안)
    float pitch = 0.0f; // Y축 회전
    float yaw   = 0.0f; // Z축 회전

    Attitude() = default;
    Attitude(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}

    void reset() {
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
};

// 3. IMU 종합 데이터 패킷 (원시 데이터 관리용)
struct ImuData {
    Vector3f    acc;                // 가속도 데이터 (g 또는 m/s^2)
    Vector3f    gyro;               // 자이로 데이터 (deg/s 또는 rad/s)
    float       temperature = 0.0f; // 센서 온도 (필요 시)
    uint64_t    timestamp = 0;      // 데이터 획득 시간 (FreeRTOS 틱 또는 마이크로초)
};
