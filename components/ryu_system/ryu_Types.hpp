/**
 * @file ryu_DroneTypes.hpp
 * @author your name (you@domain.com)
 * @brief 
 *      1. 시스템에서 사용되어지는 데이터 타입선언.
 *      2. Vector3f : 시스템에서 가장 많이 사용되어지는 데이터 타입 
 * @version 0.1
 * @date 2026-05-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <cstdint>
#include <cmath>

namespace Utils{
    enum class CoordSystem; 
}




// 1. 센서 3축 데이터 (가속도, 자이로 공용)
struct Vector3f {
    union {
        float data[3];
        struct{
            float x ;
            float y ;
            float z ;
        };
    };
     // ★ [해결책] 이 코드를 클래스/구조체 내부에 추가해 줍니다.
    constexpr Vector3f(const Vector3f&) = default; 
    // 만약 이동 생성자 경고까지 대비하려면 아래 줄도 함께 넣어주면 안전합니다.
    //constexpr Vector3f& operator=(const Vector3f&) = default;

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
    Vector3f operator*(const Vector3f& other) const {
        return Vector3f(x * other.x, y * other.y, z * other.z);
    }
    Vector3f operator/(int scalar) const {
        return Vector3f(x / scalar, y / scalar, z / scalar);
    }
    Vector3f operator/(float scalar) const {
        return Vector3f(x / scalar, y / scalar, z / scalar);
    }
   // 4. 안전한 대입 연산자 (자기 대입 방어 코드 포함)
    Vector3f& operator=(const Vector3f& other) {
        if (this != &other) { 
            x = other.x; 
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    Vector3f& operator=(float scalar) {
        x = scalar; y = scalar; z = scalar;
        return *this;
    }
     // [덤] 배열처럼 인덱스로 접근하고 싶을 때를 위한 연산자 추가 (예: vec[0] -> x축)
    float operator[](int index) const { return data[index]; }
    float& operator[](int index) { return data[index]; }

    Vector3f& norm(){
        float norm = sqrtf( x * x + y * y + z * z);
        if (norm > 0.0f) {
            *this = *this /norm;
        }
        return *this;
    }
};


template <typename T>
struct Vector{
    union {
        T data[3];
        struct{
            T x ;
            T y ;
            T z ;
        };
    };
};


// 2. 오일러 각도 기반 드론 자세 데이터
struct Attitude {
    union{
        float data[3] ={0.0f,};
        struct{
            float roll  ; // X축 회전 (도 또는 라디안)
            float pitch ; // Y축 회전
            float yaw   ; // Z축 회전
        };
    };

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

    // ★ 지자계(Mag) 독립 파트 추가
    uint64_t mag_timestamp;   // 지자계 데이터가 '실제 갱신된' 시점 (us)
    Vector3f mag;
    bool is_mag_updated;      // 이번 루프에 지자계 새 데이터가 들어왔는지 여부 (플래그)

    Utils::CoordSystem current_coordSystem;
};

struct BaroData {
    uint64_t timestamp;
    float pressure;    // 기압 (hPa)
    float altitude;    // 계산된 상대 고도 (m)
    float temperature;
    bool is_updated;
};