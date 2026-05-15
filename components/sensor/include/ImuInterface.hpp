// components/hardware/include/IIMU.hpp
#pragma once
#include "DroneTypes.hpp"

class ImuInterface {
public:
    virtual ~ImuInterface() = default;

    /**
     * @brief 센서 초기화 및 통신 연결 테스트
     * @return true 초기화 성공, false 실패
     */
    virtual bool initialize() = 0;

    /**
     * @brief 9축 센서 데이터(가속도, 자이로, 지자계) 통합 갱신
     * @param sample 데이터를 저장할 구조체 참조자
     * @return true 갱신 성공, false 실패
     */
    virtual bool updateSample(ImuData& sample) = 0;
};
