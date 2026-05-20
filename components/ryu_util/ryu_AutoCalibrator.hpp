/**
 * @file ryu_AutoCalibrator.hpp
 * @brief 
 *      1. 기체가 수평인 정지 상태에서 센서가 기판에 어떤 각도로 오장착/회전 장착 되었는지 
 *         중력 벡터를 분석하여 자동으로 식별하고 FrameTransformer용 상수를 반환합니다.
 * @version 0.3
 * @date 2026-05-18
 */
#pragma once

#include <cmath>
#include "ryu_Types.hpp"
#include "ryu_FrameTransformer.hpp"

namespace Utils {

class AutoCalibrator {
private:
    // [수정] 매크로 내부에서 안전하게 치환될 수 있도록 정적 상수로 태그 문자열 선언 완료
    static constexpr const char* TAG = "AutoCalibrator";    
public:
    /**
     * @brief 기체가 지면에 평평하게 수평을 잡고 있는 상태에서 원본 가속도 센서값을 받아 장착 방향을 역추적합니다.
     * @param raw_acc_sample 변환/정렬이 전혀 되지 않은 순수 센서 고유의 정지 상태 가속도 데이터
     * @return SensorRotation 자동으로 식별된 장착 방향 기하학 상수
     */
    static SensorRotation detect_orientation(const Vector3f& raw_acc_sample);
};

} // namespace Utils
