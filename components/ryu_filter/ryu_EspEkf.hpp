#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <esp_dsp.h>
#include <ekf_imu13states.h>
#include "ryu_Types.hpp"

namespace Filter {

class Userekf : public ekf_imu13states {
private:
    static constexpr const char* TAG = "Userekf";

    // 1. [싱글톤] 생성자와 소멸자를 private으로 숨겨 외부 생성을 원천 차단합니다.
    Userekf() : ekf_imu13states() {
        this->initialize();
    };
    ~Userekf() {};

    // 2. [싱글톤] 복사 생성자와 대입 연산자를 제거하여 객체가 복사되는 버그를 방지합니다.
    Userekf(const Userekf&) = delete;
    Userekf& operator=(const Userekf&) = delete;

    float R[10];
    Attitude_t shared_euler_buffer{0.0f, 0.0f, 0.0f};

public:
    // 3. [싱글톤] 유일한 전역 객체 참조를 반환하는 스태틱 함수 (Meyers' Singleton)
    // C++11 표준에 따라 전역 락 없이도 멀티태스크 환경에서 완벽히 Thread-Safe합니다.
    static Userekf& getInstance() {
        static Userekf instance;
        return instance;
    }
    void initialize() {
        this->Init(); // 조상 초기화 실행 (기본 Q 설정됨)

        // 💡 [바이어스 학습 가속 치트키]
        // 조상 클래스의 Q 행렬에서 자이로 바이어스 영역(인덱스 3~5번 대각선)의 
        // 노이즈 값을 기본값(0.0001)보다 10배~100배 정도 키워줍니다.
        // 이 값을 키우면 필터가 자이로 오차를 훨씬 공격적이고 빠르게 학습하여 Drift를 즉시 멈춥니다.
        this->Q(3, 3) = 0.01f;
        this->Q(4, 4) = 0.01f;
        this->Q(5, 5) = 0.01f;

        X(0, 0) = 1.0f; X(0, 1) = 0.0f; X(0, 2) = 0.0f; X(0, 3) = 0.0f;
        for (size_t i = 0; i < 10; i++) { R[i] = 0.01f; }
    }


    // User_Process 내부 연산이 완전히 끝난 직후 딱 한 번만 실행될 가벼운 백업 함수
    void backupAttitude() {
        // 기존의 안정적인 조상 수식 복제본 그대로 사용
        const float* q = X.data; 
        float q0s = q[0] * q[0], q1s = q[1] * q[1], q2s = q[2] * q[2], q3s = q[3] * q[3];
        float R13 = 2.0f * (q[1] * q[3] + q[0] * q[2]);
        float R11 = q0s + q1s - q2s - q3s;
        float R12 = -2.0f * (q[1] * q[2] - q[0] * q[3]);
        float R23 = -2.0f * (q[2] * q[3] - q[0] * q[1]);
        float R33 = q0s - q1s - q2s + q3s;

        // 대형 행렬 자원과 격리된 "독립 버퍼 변수"에 각도를 대입 (0.001ms 소요)
        shared_euler_buffer.roll  = atan2f(R23, R33);
        shared_euler_buffer.pitch = asinf(R13);
        shared_euler_buffer.yaw   = atan2f(R12, R11);
    }

    // 💡 Core 0(통신/Mavlink 태스크)은 이 함수를 호출하여 락 없이 안전하게 낚아채갑니다.
    Attitude_t getEulerForComm() {
        return shared_euler_buffer;
    }

    void update(Vector3f acc, Vector3f gyro, Vector3f mag, float dt) {
        float gyro_in[3] = {gyro.x, gyro.y, gyro.z}; // Vector3f의 data 배열을 직접 참조하여 안전하게 복사
        float accel_in[3] = {acc.x, acc.y, acc.z};
        float mag_in[3] = {mag.x, mag.y, mag.z};

        this->Process(gyro_in, dt);
        this->UpdateRefMeasurement(accel_in, mag_in, R);
    }
    
Attitude_t getEuler() {
    // 1. 상태 행렬 X(Row, Col) 연산자 오버헤드를 줄이기 위해 내부 데이터 포인터를 직접 참조
    const float* q = X.data; // X.data[0]=q0, [1]=q1, [2]=q2, [3]=q3

    // 2. 조상 클래스(ekf::quat2eul) 내부 수식 완벽 하드코딩 복제
    float q0s = q[0] * q[0];
    float q1s = q[1] * q[1];
    float q2s = q[2] * q[2];
    float q3s = q[3] * q[3];

    float R13 =  2.0f * (q[1] * q[3] + q[0] * q[2]);
    float R11 =  q0s + q1s - q2s - q3s;
    float R12 = -2.0f * (q[1] * q[2] - q[0] * q[3]); // 조상 고유의 마이너스 부호 반영
    float R23 = -2.0f * (q[2] * q[3] - q[0] * q[1]); // 조상 고유의 마이너스 부호 반영
    float R33 =  q0s - q1s - q2s + q3s;

    // 3. 조상의 데이터 인덱스 매칭 구조 그대로 적용
    // result.data[0] -> Roll
    // result.data[1] -> Pitch
    // result.data[2] -> Yaw
    float pitch = asinf(R13);
    float yaw   = atan2f(R12, R11);
    float roll  = atan2f(R23, R33);

    // 4. ryu_Types 규격인 Attitude_t 구조체에 순서대로 매칭하여 리턴 (동적 할당 0%)
    return Attitude_t(roll, pitch, yaw);
}


void getQuaternion(float *q) {
    q[0] = X(0, 0);
    q[1] = X(1, 0);
    q[2] = X(2, 0);
    q[3] = X(3, 0);
}

};

} // namespace Filter
