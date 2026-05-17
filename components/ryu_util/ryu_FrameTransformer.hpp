/**
 * @file ryu_FrameTransformer.hpp
 * @author your name (you@domain.com)
 * @brief 
 *      1. 데이터의 ENU방식과 NED방식의 변환
 * @version 0.1
 * @date 2026-05-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once
#include "ryu_DroneTypes.hpp" // Vector3f 및 ImuData 구조체가 정의된 헤더

namespace Utils {

enum class CoordSystem { NED, ENU };

enum class SensorRotation {
    ROTATION_NONE,        // 기체 정면과 칩 정면이 일치
    YAW_90,               // 시계방향 90도 회전 장착
    YAW_180,              // 뒤집어서 장착 (180도 회전)
    YAW_270,              // 반시계방향 90도 회전 장착
    ROLL_180,             // 보드가 뒤집혀 아랫면을 봄 (가장 흔함)
    ROLL_180_YAW_90       // 보드가 뒤집히고 90도 회전됨
};

class FrameTransformer {
private:
    static constexpr const char* TAG = "FrameTransformer";    
public:
    // 단독 유틸리티 함수로 선언하여 객체 생성 없이도 쓸 수 있게 설계
    static void align_to_body(ImuData& src, SensorRotation rotation) {
        // 가속도 변환
        Vector3f aligned_acc = transform_vector(src.acc, rotation);
        // 자이로 변환
        Vector3f aligned_gyro = transform_vector(src.gyro, rotation);

        // 결과 반영
        src.acc = aligned_acc;
        src.gyro = aligned_gyro;
    }
    /**
     * @brief 순수 센서 데이터를 상위 제어기가 원하는 목적 좌표계로 정렬 및 부호 변환합니다.
     * @param src 센서 칩에서 갓 읽어온 순수 변환 데이터 (물리 단위 완료 상태)
     * @param target_system 상위 칼만/PID가 사용하는 좌표계 (NED 또는 ENU)
     */
    static void convert_to(ImuData& src, CoordSystem target_system) {
if (src.current_coordSystem == target_system) return;

    // 가속도, 자이로, 지자계 모두 동일한 축 변환 규칙을 따릅니다.
    auto swap_and_invert = [](Vector3f& v) {
        float old_x = v.x; float old_y = v.y; float old_z = v.z;
        v.x = old_y;       v.y = old_x;       v.z = -old_z;
    };

    if ((src.current_coordSystem == CoordSystem::NED && target_system == CoordSystem::ENU) ||
        (src.current_coordSystem == CoordSystem::ENU && target_system == CoordSystem::NED)) {
        
        swap_and_invert(src.acc);
        swap_and_invert(src.gyro);
        swap_and_invert(src.mag); // 지자계 데이터 구조체에 추가 필요

        src.current_coordSystem = target_system;
    }
    }
private:
    static Vector3f transform_vector(const Vector3f& vec, SensorRotation rotation) {
        Vector3f out = vec;
        switch (rotation) {
            case SensorRotation::YAW_90:
                out.x = -vec.y;
                out.y = vec.x;
                break;
            case SensorRotation::YAW_180:
                out.x = -vec.x;
                out.y = -vec.y;
                break;
            case SensorRotation::YAW_270:
                out.x = vec.y;
                out.y = -vec.x;
                break;
            case SensorRotation::ROLL_180:
                out.y = -vec.y;
                out.z = -vec.z;
                break;
            case SensorRotation::ROLL_180_YAW_90:
                out.x = vec.y;
                out.y = vec.x;
                out.z = -vec.z;
                break;
            default: // ROTATION_NONE
                break;
        }
        return out;
    }
};

} // namespace Utils
