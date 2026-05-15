#pragma once
#include "DroneTypes.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace Utils {

class SharedDataManager {
private:
    ImuData     _shared_imu_data;
    Attitude    _currentAttitude;
    Attitude    _targetAttitude; 
    BaroData _shared_baro_data;
    
    // ★ 안전성 핵심: 뮤텍스를 3개로 분리하여 태스크 간 병목을 완벽히 격리합니다.
    SemaphoreHandle_t _mutex_imu;
    SemaphoreHandle_t _mutex_current;
    SemaphoreHandle_t _mutex_target;
    SemaphoreHandle_t _mutex_baro ;

public:
    SharedDataManager() {
        _mutex_imu     = xSemaphoreCreateMutex();
        _mutex_current = xSemaphoreCreateMutex();
        _mutex_target  = xSemaphoreCreateMutex();
        _mutex_baro = xSemaphoreCreateMutex();
    }

    // 1. Core 0 (SensorTask)가 데이터를 채워 넣는 창구
    void update_latest_imu(const ImuData& new_data) {
        // 무한 대기(portMAX_DELAY) 대신 가벼운 대기 시간을 주어 센서 루프 타이밍을 절대적으로 보호합니다.
        if (xSemaphoreTake(_mutex_imu, pdMS_TO_TICKS(1)) == pdTRUE) {
            _shared_imu_data = new_data; 
            xSemaphoreGive(_mutex_imu);
        }
    }

    // 2. Core 1 (ControlTask)가 제어 연산 직전 데이터를 꺼내가는 창구
    bool get_latest_imu(ImuData& out_data) {
        if (xSemaphoreTake(_mutex_imu, pdMS_TO_TICKS(1)) == pdTRUE) {
            out_data = _shared_imu_data;
            xSemaphoreGive(_mutex_imu);
            return true;
        }
        return false; 
    }

    // 독립된 뮤텍스를 사용하므로 조종기 입력을 받는 도중에도 IMU와 PID 연산은 끊김 없이 독립 구동됩니다.
    void setAttitude(const Attitude& att) {
        if (xSemaphoreTake(_mutex_current, pdMS_TO_TICKS(1)) == pdTRUE) {
            _currentAttitude = att;
            xSemaphoreGive(_mutex_current);
        }
    }

    void setTargetAttitude(const Attitude& target) {
        if (xSemaphoreTake(_mutex_target, pdMS_TO_TICKS(1)) == pdTRUE) {
            _targetAttitude = target;
            xSemaphoreGive(_mutex_target);
        }
    }

    Attitude getAttitude() {
        Attitude temp {};
        if (xSemaphoreTake(_mutex_current, pdMS_TO_TICKS(1)) == pdTRUE) {
            temp = _currentAttitude;
            xSemaphoreGive(_mutex_current);
        }
        return temp;
    }

    Attitude getTargetAttitude() {
        Attitude temp {};
        if (xSemaphoreTake(_mutex_target, pdMS_TO_TICKS(1)) == pdTRUE) {
            temp = _targetAttitude;
            xSemaphoreGive(_mutex_target);
        }
        return temp;
    }
    void update_latest_baro(const BaroData& new_data) {
        if (xSemaphoreTake(_mutex_baro, pdMS_TO_TICKS(1)) == pdTRUE) {
            _shared_baro_data = new_data;
            _shared_baro_data.is_updated = true;
            xSemaphoreGive(_mutex_baro);
        }
    }

    bool get_latest_baro(BaroData& out_data) {
        if (xSemaphoreTake(_mutex_baro, pdMS_TO_TICKS(1)) == pdTRUE) {
            out_data = _shared_baro_data;
            _shared_baro_data.is_updated = false; // 소모 완료 플래그 리셋
            xSemaphoreGive(_mutex_baro);
            return true;
        }
        return false;
    }
};



} // namespace Utils
