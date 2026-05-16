#pragma once
#include <cmath>
#include <esp_timer.h>
#include "ImuInterface.hpp"

namespace Sensor{

class VirtualIMU : public Interface::IImuSensor {
public:
    VirtualIMU(){};
    esp_err_t initialize() override { return ESP_OK; }
    esp_err_t deinitialize()  override { return ESP_OK;}
    esp_err_t updateSample(ImuData& sample) override {
        // 실제 센서 대신 수학식(정현파)으로 흔들림 데이터를 강제로 주입
        sample.acc.x = sin(esp_timer_get_time() / 1000000.0f);
        sample.acc.y = cos(esp_timer_get_time() / 1000000.0f);
        sample.acc.z = 1.0f;
        return ESP_OK;
    }
};

}
