
#pragma once
#include "DroneTypes.hpp"
#include <esp_err.h>


namespace Interface{

// 각 sensor는 해당 인터페이스를 계승 받아서 사용.
class IImuSensor {
public:
    virtual ~IImuSensor() = default;

    /**
     * @brief 센서 초기화 및 통신 연결 테스트
     * @return true 초기화 성공, false 실패
     */
    virtual esp_err_t initialize() = 0;
    virtual esp_err_t deinitialize() =0;

    /**
     * @brief 9축 센서 데이터(가속도, 자이로, 지자계) 통합 갱신
     * @param sample 데이터를 저장할 구조체 참조자
     * @return true 갱신 성공, false 실패
     */
    virtual bool updateSample(ImuData& sample) = 0;
};


}// namespace IImuSensor

// // 테스트용 가짜 센서 클래스
// class VirtualIMU : public IIMU {
// public:
//     bool initialize() override { return true; }
//     bool updateSample(IMUSample& sample) override {
//         // 실제 센서 대신 수학식(정현파)으로 흔들림 데이터를 강제로 주입
//         sample.accel.x = sin(esp_timer_get_time() / 1000000.0f);
//         return true;
//     }
// };