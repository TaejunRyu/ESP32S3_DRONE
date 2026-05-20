#include "ryu_AutoCalibrator.hpp"

#include <esp_log.h>

namespace Utils{


SensorRotation AutoCalibrator::detect_orientation(const Vector3f &raw_acc_sample)
{
    // 정지 상태 중력 가속도(약 1.0g)가 어느 축에 실려있는지 판별하기 위한 임계값
    const float threshold = 0.7f; 
    
    float ax = raw_acc_sample.x;
    float ay = raw_acc_sample.y;
    float az = raw_acc_sample.z;

    // 1. Z축에 중력이 감지된 경우 (일반적인 수평 장착 라인)
    if (std::abs(az) > threshold) {
        // [물리 법칙 교정]: 정방향 장착 시 중력 반작용으로 인해 가속도계는 하늘 방향인 음수(-1.0g)를 뱉습니다.
        if (az < 0.0f) {
            ESP_LOGI(TAG, "정방향 평평하게 장착됨 완료 (ROTATION_NONE)");
            return SensorRotation::ROTATION_NONE;
        } else {
            // 보드가 뒤집혀 장착되면 중력 반작용이 센서 기준 Down 방향이 되므로 양수(+1.0g)가 나옵니다.
            ESP_LOGI(TAG, "보드가 뒤집혀서 아랫면을 보고 장착됨 감지 (ROLL_180)");
            return SensorRotation::ROLL_180;
        }
    }
    
    // 2. X축에 중력이 감지된 경우 (칩이 90도 세워져 수직 장착된 경우 등)
    if (std::abs(ax) > threshold) {
        if (ax < 0.0f) {
            ESP_LOGI(TAG, "보드가 기수 우측 방향으로 90도 회전되어 세워짐 감지 (PITCH_90)");
            return SensorRotation::PITCH_90; 
        } else {
            ESP_LOGI(TAG, "보드가 기수 좌측 방향으로 90도 회전되어 세워짐 감지 (PITCH_270)");
            return SensorRotation::PITCH_270;
        }
    }

    // 3. Y축에 중력이 감지된 경우 (기수 전방/후방 수직 세움 장착)
    if (std::abs(ay) > threshold) {
        if (ay < 0.0f) {
            ESP_LOGI(TAG, "보드가 전방을 바라보고 수직으로 세워짐 감지 (ROLL_90)");
            return SensorRotation::ROLL_90;
        } else {
            ESP_LOGI(TAG, "보드가 후방을 바라보고 수직으로 세워짐 감지 (ROLL_270)");
            return SensorRotation::ROLL_270;
        }
    }

    // 판별 실패 시 예외 처리 기본값 반환
    ESP_LOGW(TAG, "방향 식별 불가능 (흔들림 유입), 기본 정방향(ROTATION_NONE)으로 고정합니다.");
    return SensorRotation::ROTATION_NONE;
}




}