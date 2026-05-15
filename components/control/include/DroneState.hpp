#pragma once
#include "DroneTypes.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class DroneState {
private:
    // 실제 데이터 공간 (외부에서 직접 접근 불가)
    ImuData     _rawIMU;
    Attitude    _currentAttitude;
    Attitude    _targetAttitude; // 조종기 등으로부터 입력받은 목표 자세

    // 멀티태스킹 환경(FreeRTOS)에서 데이터가 깨지는 것을 막기 위한 Mutex 락
    SemaphoreHandle_t _mutex;

public:
    DroneState() {
        _mutex = xSemaphoreCreateMutex();
    }

    ~DroneState() {
        vSemaphoreDelete(_mutex);
    }

    // --- Thread-Safe Setter ---
    void setImuData(const ImuData& data) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _rawIMU = data;
            xSemaphoreGive(_mutex);
        }
    }

    void setAttitude(const Attitude& att) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _currentAttitude = att;
            xSemaphoreGive(_mutex);
        }
    }

    void setTargetAttitude(const Attitude& target) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _targetAttitude = target;
            xSemaphoreGive(_mutex);
        }
    }

    // --- Thread-Safe Getter ---
    ImuData getImuData() {
        ImuData temp;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            temp = _rawIMU;
            xSemaphoreGive(_mutex);
        }
        return temp;
    }

    Attitude getAttitude() {
        Attitude temp;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            temp = _currentAttitude;
            xSemaphoreGive(_mutex);
        }
        return temp;
    }

    Attitude getTargetAttitude() {
        Attitude temp;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            temp = _targetAttitude;
            xSemaphoreGive(_mutex);
        }
        return temp;
    }
};
