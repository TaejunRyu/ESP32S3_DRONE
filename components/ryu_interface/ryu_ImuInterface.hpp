
#pragma once
#include "ryu_DroneTypes.hpp"
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
    virtual esp_err_t updateSample(ImuData& sample) = 0;
};


}// namespace IImuSensor

