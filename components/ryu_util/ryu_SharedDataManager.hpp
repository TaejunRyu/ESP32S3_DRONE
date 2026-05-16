/**
 * @file ryu_SharedDataManager.hpp
 * @brief 
 *      1. Singleton으로 설계하여 필요한 데이터를 바로 가져 갈수 있도록한다.
 *      2. 현재 세마포어를 사용하고 있으며 락 획득 실패 시 이전 데이터를 안전하게 유지합니다.
 *      3. 센서 데이터와 euler(attitude)를 저장한다. 
 * @version 0.2
 * @date 2026-05-16
 */

#pragma once
#include "ryu_DroneTypes.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace Utils {

class SharedDataManager {
    private:
        // 싱글톤 강제를 위해 생성자와 소멸자를 모두 private으로 격리
        SharedDataManager() {
            _mutex_imu     = xSemaphoreCreateMutex();
            _mutex_current = xSemaphoreCreateMutex();
            _mutex_target  = xSemaphoreCreateMutex();
            _mutex_baro    = xSemaphoreCreateMutex();
        }
        ~SharedDataManager() {
            // 뮤텍스 자원 해제 (안전 장치)
            if (_mutex_imu)     vSemaphoreDelete(_mutex_imu);
            if (_mutex_current) vSemaphoreDelete(_mutex_current);
            if (_mutex_target)  vSemaphoreDelete(_mutex_target);
            if (_mutex_baro)    vSemaphoreDelete(_mutex_baro);
        }

        // 실제 공유 데이터 저장소
        ImuData     _shared_imu_data   {};
        Attitude    _currentAttitude   {};
        Attitude    _targetAttitude    {}; 
        BaroData    _shared_baro_data  {};
        
        // 데이터 파트별 독립 뮤텍스
        SemaphoreHandle_t _mutex_imu     = nullptr;
        SemaphoreHandle_t _mutex_current = nullptr;
        SemaphoreHandle_t _mutex_target  = nullptr;
        SemaphoreHandle_t _mutex_baro    = nullptr;

    public:
        // 싱글톤 인스턴스 반환 구문
        static SharedDataManager& getinstance() {
            static SharedDataManager instance;
            return instance;
        }
        
        // 복사 및 대입 금지
        SharedDataManager(const SharedDataManager&) = delete;
        SharedDataManager& operator=(const SharedDataManager&) = delete;
        SharedDataManager& operator=(SharedDataManager&&) = delete;

        // --- 1. IMU 데이터 창구 ---
        void update_latest_imu(const ImuData& new_data) {
            // 센서 태스크 주기를 보호하기 위해 대기 시간은 0 또는 최소화(1개 틱)
            if (xSemaphoreTake(_mutex_imu, 0) == pdTRUE) {
                _shared_imu_data = new_data; 
                xSemaphoreGive(_mutex_imu);
            }
        }

        bool get_latest_imu(ImuData& out_data) {
            // 비행 제어 태스크가 블로킹되지 않도록 대기 시간은 '0' 사용
            if (xSemaphoreTake(_mutex_imu, 0) == pdTRUE) {
                out_data = _shared_imu_data;
                xSemaphoreGive(_mutex_imu);
                return true;
            }
            return false; // 실패 시 호출 측에서 out_data(이전 값)를 그대로 유지하도록 유도
        }

        // --- 2. 현재 자세(Attitude) 데이터 창구 ---
        void setAttitude(const Attitude& att) {
            if (xSemaphoreTake(_mutex_current, 0) == pdTRUE) {
                _currentAttitude = att;
                xSemaphoreGive(_mutex_current);
            }
        }

        Attitude getAttitude() {
            Attitude temp {};
            if (xSemaphoreTake(_mutex_current, 0) == pdTRUE) {
                temp = _currentAttitude;
                xSemaphoreGive(_mutex_current);
                return temp;
            }
            // 락 획득 실패 시, 쓰레기 값이 아닌 클래스 내부의 직전 최신 데이터를 안전하게 복사하여 반환
            return _currentAttitude; 
        }

        // --- 3. 목표 자세(Target Attitude) 데이터 창구 ---
        void setTargetAttitude(const Attitude& target) {
            if (xSemaphoreTake(_mutex_target, 0) == pdTRUE) {
                _targetAttitude = target;
                xSemaphoreGive(_mutex_target);
            }
        }

        Attitude getTargetAttitude() {
            if (xSemaphoreTake(_mutex_target, 0) == pdTRUE) {
                Attitude temp = _targetAttitude;
                xSemaphoreGive(_mutex_target);
                return temp;
            }
            return _targetAttitude; // 락 실패 시 직전 타겟 값 유지
        }

        // --- 4. 고도(Baro) 데이터 창구 ---
        void update_latest_baro(const BaroData& new_data) {
            if (xSemaphoreTake(_mutex_baro, 0) == pdTRUE) {
                _shared_baro_data = new_data;
                _shared_baro_data.is_updated = true;
                xSemaphoreGive(_mutex_baro);
            }
        }

        bool get_latest_baro(BaroData& out_data) {
            if (xSemaphoreTake(_mutex_baro, 0) == pdTRUE) {
                out_data = _shared_baro_data;
                _shared_baro_data.is_updated = false; // 소모 완료 플래그 리셋
                xSemaphoreGive(_mutex_baro);
                return true;
            }
            return false;
        }
};

} // namespace Utils
