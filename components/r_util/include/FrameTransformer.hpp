#pragma once
#include "DroneTypes.hpp" // Vector3f 및 ImuData 구조체가 정의된 헤더

namespace Utils {

enum class SensorRotation {
    ROTATION_NONE,        // 기체 정면과 칩 정면이 일치
    YAW_90,               // 시계방향 90도 회전 장착
    YAW_180,              // 뒤집어서 장착 (180도 회전)
    YAW_270,              // 반시계방향 90도 회전 장착
    ROLL_180,             // 보드가 뒤집혀 아랫면을 봄 (가장 흔함)
    ROLL_180_YAW_90       // 보드가 뒤집히고 90도 회전됨
};

class FrameTransformer {
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
