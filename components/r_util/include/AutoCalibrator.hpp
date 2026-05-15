#pragma once
#include <cmath>
#include <esp_log.h>
#include "DroneTypes.hpp"
#include "FrameTransformer.hpp"

namespace Utils {

class AutoCalibrator {
private:
    static constexpr const char* TAG = "AutoCalibrator";    
public:
    /**
     * @brief 기체가 수평인 상태에서 센서 고유의 리딩 값을 받아와 장착 방향을 역추적합니다.
     * @param raw_acc_sample 변환/정렬이 전혀 되지 않은 순수 센서 고유 가속도 데이터
     * @return SensorRotation 자동으로 식별된 장착 방향 상수
     */
    static SensorRotation detect_orientation(const Vector3f& raw_acc_sample) {
        // 중력 가속도(약 1.0g)가 어느 축에 실려있는지 임계값(0.7g)으로 판별
        const float threshold = 0.7f; 
        
        float ax = raw_acc_sample.x;
        float ay = raw_acc_sample.y;
        float az = raw_acc_sample.z;

        // 1. Z축에 중력이 걸린 경우 (가장 표준적인 수평 장착 상태)
        if (std::abs(az) > threshold) {
            if (az > 0) {
                ESP_LOGI("AUTO_CAL", "정방향 평평하게 장착됨 (ROTATION_NONE)");
                return SensorRotation::ROTATION_NONE;
            } else {
                ESP_LOGI("AUTO_CAL", "보드가 뒤집혀서 아랫면을 보고 장착됨 (ROLL_180)");
                return SensorRotation::ROLL_180;
            }
        }
        
        // 2. X축에 중력이 걸린 경우 (칩이 90도 기울어져 수직 장착된 경우 등)
        if (std::abs(ax) > threshold) {
            if (ax > 0) {
                ESP_LOGI("AUTO_CAL", "칩이 우측으로 90도 회전되어 수직 장착됨");
                return SensorRotation::YAW_90; // 실제 기판 배치 상황에 맞춰 매핑 변경 가능
            } else {
                return SensorRotation::YAW_270;
            }
        }

        // 3. Y축에 중력이 걸린 경우
        if (std::abs(ay) > threshold) {
            if (ay > 0) {
                return SensorRotation::YAW_180;
            }
        }

        // 판별 실패 시 기본값 반환
        ESP_LOGW("AUTO_CAL", "방향 식별 실패, 기본 정방향으로 설정합니다.");
        return SensorRotation::ROTATION_NONE;
    }
};

} // namespace Utils


// // 1. 센서 자체 초기화 완료
// icm20948.initialize(); 

// ESP_LOGI("MAIN", "드론을 평평한 바닥에 두고 움직이지 마세요. 1초 뒤 자동 축 정렬을 시작합니다.");
// vTaskDelay(pdMS_TO_TICKS(1000));

// // 2. 정지 상태에서 순수 센서 원시 데이터 샘플링 (10번 읽어 평균 내면 더 정확함)
// ImuData raw_sample = {0};
// // 아직 정렬 상수를 모르므로 정렬 전 순수 데이터 수신을 위해 임시로 READ 진행
// icm20948.read_data(raw_sample); 

// // 3. 자동 추출 로직 호출하여 회전 상태 값 획득
// Utils::SensorRotation detected_rot = Utils::AutoCalibrator::detect_orientation(raw_sample.acc);

// // 4. 추출된 회전 값을 센서 관리자나 시스템 전역 설정 변수에 저장하여 이후 매 루프마다 자동 적용
// // 예: icm20948.set_applied_rotation(detected_rot);
