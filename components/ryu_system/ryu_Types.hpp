/**
 * @file ryu_Types.hpp
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


    Vector3f& normalize(){
        float norm = sqrtf( x * x + y * y + z * z);
        if (norm > 0.0f) {
            *this = *this / norm;
        }
        return *this;
    }
};

// float가 아니 다른 변수들의 활용을 위함. 
template <typename T>
struct Vector3{
    union {
        T data[3];
        struct{
            T x ;
            T y ;
            T z ;
        };
    };
    constexpr Vector3(const Vector3&) = default;
    Vector3() = default;
    Vector3( T _x, T _y, T _z) : x(_x), y(_y), z(_z) {} 
    // 연산자 오버로딩: 센서 오프셋이나 필터 연산을 직관적으로 만들기 위함
    Vector3<T> operator+(const Vector3<T>& other) const {
        return Vector3<T>(x + other.x, y + other.y, z + other.z);
    }
    Vector3<T> operator-(const Vector3<T>& other) const {
        return Vector3<T>(x - other.x, y - other.y, z - other.z);
    }

    Vector3<T> operator*(float scalar) const {
        return Vector3<T>(x * scalar, y * scalar, z * scalar);
    }
    Vector3<T> operator*(const Vector3<T>& other) const {
        return Vector3<T>(x * other.x, y * other.y, z * other.z);
    }
    Vector3<T> operator/(int scalar) const {
        return Vector3<T>(x / scalar, y / scalar, z / scalar);
    }
    Vector3<T> operator/(float scalar) const {
        return Vector3<T>(x / scalar, y / scalar, z / scalar);
    }
   // 4. 안전한 대입 연산자 (자기 대입 방어 코드 포함)
    Vector3<T>& operator=(const Vector3<T>& other) {
        if (this != &other) { 
            x = other.x; 
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    Vector3<T>& operator=(float scalar) {
        x = scalar; y = scalar; z = scalar;
        return *this;
    }
     // [덤] 배열처럼 인덱스로 접근하고 싶을 때를 위한 연산자 추가 (예: vec[0] -> x축)
    T operator[](int index) const { return data[index]; }
    T& operator[](int index) { return data[index]; }


    Vector3<T>& normalize(){
        float norm = sqrtf( x * x + y * y + z * z);
        if (norm > 0.0f) {
            *this = *this / norm;
        }
        return *this;
    }
};


// 2. 오일러 각도 기반 드론 자세 데이터
struct Attitude_t {
    union{
        float data[3] ={0.0f,};
        struct{
            float roll  ; // X축 회전 (도 또는 라디안)
            float pitch ; // Y축 회전
            float yaw   ; // Z축 회전
        };
    };
 
    Attitude_t() = default;
    Attitude_t(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
    constexpr Attitude_t(const Attitude_t&) = default; 
   // 4. 안전한 대입 연산자 (자기 대입 방어 코드 포함)
    Attitude_t& operator=(const Attitude_t& other) {
        if (this != &other) { 
            roll    = other.roll; 
            pitch   = other.pitch;
            yaw     = other.yaw;
        }
        return *this;
    }
    Attitude_t operator*(float scalar) const {
        return Attitude_t(roll * scalar, pitch * scalar, yaw * scalar);
    }

    void reset() {
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
};


struct QgcAttitude_t{
    Attitude_t att;
    Vector3f   speed;
    float      alt;       // 💡 [추가] 수직 칼만 필터 추정 고도 (m)
    float      v_speed;   // 💡 [추가] 수직 칼만 필터 추정 수직 속도 (m/s)
    float base_throttle;
};


// 3. IMU 종합 데이터 패킷 (원시 데이터 관리용)
struct SensorData {
    Vector3f    acc;                // 가속도 데이터 (g 또는 m/s^2)
    Vector3f    gyro;               // 자이로 데이터 (deg/s 또는 rad/s)
    //float       temperature = 0.0f; // 센서 온도 (필요 시)
    //uint64_t    timestamp = 0;      // 데이터 획득 시간 (FreeRTOS 틱 또는 마이크로초)
    // ★ 지자계(Mag) 독립 파트 추가
    //uint64_t mag_timestamp;   // 지자계 데이터가 '실제 갱신된' 시점 (us)
    Vector3f mag;
    bool     is_mag_updated;      // 이번 루프에 지자계 새 데이터가 들어왔는지 여부 (플래그)
};


/**
 * @brief 
 * 
 */
struct BaroData {
    uint64_t timestamp;
    float gnd_pressure;    // 기압 (hPa)
    float pressure;    // 기압 (hPa)
    float altitude;    // 계산된 상대 고도 (m)
    //float temperature;
    //float climb_rate;
};

        // --- GPS 사용자 데이터 ---
struct gps_data_t {
    uint32_t    iTOW;           // gps 시간
    int         date;           // 기본 날자
    float       utc_time;       // 기본 시간 
    uint8_t     fixType;        // GPS 위치 고정 여부
    uint8_t     numSat;         // 위성 개수
    double      lat;            // 위도 (deg)
    double      lon;            // 경도 (deg)
    int32_t     height;         // 타원체 고도 (mm)    
    int32_t     horMSL;         // 해수면 고도 (mm)
    uint32_t    horAcc;         // 수평 정확도(mm)
    uint32_t    verAcc;         // 수직 정확도(mm)
    int16_t     velNorth;       // (North Velocity),  GPS정보  VGT 정보에서 뽑아야한다. 
    int16_t     velEast;        // (East Velocity), 
    int16_t     velDown;        // (Down Velocity)
    uint16_t    gndSpeed;       // 지표 속도 (mm/s)
    uint16_t    headMotion;     // 이동 방향 (Degree * 10^-5 -> Centi-Degree)
    uint32_t    speedAcc;       // 속도 정확도
    uint16_t    pDOP;           // 위치 정밀도 저하율 (0.01 단위)
    float       magDec ;        // 자기 편차 적용 (필요 시)
    float       relative_alt;   // 상대 고도 (홈 기준)
    float       home_alt = -9999.0f;       // 처음 시스템 시작시(gps가동시)의 고도  
    uint64_t    last_update_stamp;
};


// RC 데이터가 어디서 들어오는지에 따라 RC 데이터 타입을 구분하기 위한 열거형과 구조체 정의
enum RemoteControlType{
    RC_FLYSKY,
    RC_QGC
};

// 4. RC 조종 데이터 패킷
struct rc_data_t {
    float throttle;  // 스로틀 (0~100%)
    float roll;      // 롤 (-100~100)
    float pitch;     // 피치 (-100~100)
    float yaw;       // 요 (-100~100)
    float aux1;      // 보조 채널 1 (고도 유지)
    float aux2;      // 보조 채널 2
    float aux3;      // 보조 채널 3 (SWC 3단)
    float aux4;      // 보조 채널 4
    RemoteControlType type;
};
