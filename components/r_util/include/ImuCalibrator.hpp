#pragma once
#include <esp_log.h>
#include "DroneTypes.hpp"

namespace Utils {

class ImuCalibrator {
private:

    static constexpr const char* TAG = "ImuCalibrator";
     Vector3f _gyro_bias;
    Vector3f _accel_bias;
    bool _is_calibrated = false;

    const float _lpf_alpha = 0.2f; 
    Vector3f _last_filtered_accel;
    Vector3f _last_filtered_gyro;

public:
    ImuCalibrator() = default;

    void accumulate_bias(const ImuData& raw_data, int sample_count) {
        _gyro_bias = _gyro_bias + raw_data.gyro;
        _accel_bias = _accel_bias + Vector3f(raw_data.acc.x, raw_data.acc.y, raw_data.acc.z - 1.0f); 

        if (sample_count >= 500) {
            _gyro_bias = _gyro_bias * (1.0f / 500.0f);
            _accel_bias = _accel_bias * (1.0f / 500.0f);
            _is_calibrated = true;
            ESP_LOGI("IMU_CAL", "교정 완료 -> Gyro Bias X: %f, Y: %f, Z: %f", _gyro_bias.x, _gyro_bias.y, _gyro_bias.z);
        }
    }

    void apply_filter(ImuData& io_data) {
        if (!_is_calibrated) return;

        // 오프셋 차감 연산
        Vector3f clean_acc  = io_data.acc - _accel_bias;
        Vector3f clean_gyro = io_data.gyro - _gyro_bias;

        // 1차 LPF 필터링 처리
        _last_filtered_accel = (clean_acc * _lpf_alpha) + (_last_filtered_accel * (1.0f - _lpf_alpha));
        _last_filtered_gyro  = (clean_gyro * _lpf_alpha) + (_last_filtered_gyro * (1.0f - _lpf_alpha));

        // 최종 정제 데이터 반영
        io_data.acc  = _last_filtered_accel;
        io_data.gyro = _last_filtered_gyro;
    }
};

} // namespace Utils


// Utils::ImuCalibrator calibrator;

// void flight_control_loop() {
//     ImuData imu_data;
//     icm20948.updateSample(imu_data); // 1단계: 순수 데이터 취득

//     if (시스템_초기화_중) {
//         calibrator.accumulate_bias(imu_data, current_sample_count++);
//         return;
//     }

//     // [방어 레이어 가동]
//     calibrator.apply_filter(imu_data); // 0점 오차 제거 및 고주파 진동 부드럽게 커팅 완료!

//     // 2단계: 노이즈가 없는 깨끗한 데이터를 칼만 필터가 원하는 ENU 축으로 전환
//     Utils::FrameTransformer::convert_to(imu_data, Utils::TargetCoord::ENU);

//     // 3단계: 칼만 필터 연산 수행 (노이즈가 없으므로 정밀한 각도 추정 가능)
//     // kalman.update(imu_data);
// }
