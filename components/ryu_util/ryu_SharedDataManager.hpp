/**
 * @file ryu_SharedDataManager.hpp
 * @brief 
 *      1. Singleton으로 설계하여 필요한 데이터를 바로 가져 갈수 있도록한다.
 *      2. 현재 세마포어를 사용하고 있으며 락 획득 실패 시 이전 데이터를 안전하게 유지합니다.
 *      3. 센서 데이터와 euler(attitude)를 저장한다. 
 * @version 0.3
 * @date 2026-05-18
 */

#pragma once
#include "ryu_Types.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic> // 💡 멀티코어 간 정밀 동기화를 위한 원자성 헤더 추가

namespace Utils {

class SharedDataManager {
    private:
        SharedDataManager() {
            _mutex_imu     = xSemaphoreCreateMutex();
            _mutex_current = xSemaphoreCreateMutex();
            _mutex_target  = xSemaphoreCreateMutex();
            _mutex_baro    = xSemaphoreCreateMutex();
            _is_imu_calibrated.store(false); // 초기값 설정
        }
        ~SharedDataManager() {
            if (_mutex_imu)     vSemaphoreDelete(_mutex_imu);
            if (_mutex_current) vSemaphoreDelete(_mutex_current);
            if (_mutex_target)  vSemaphoreDelete(_mutex_target);
            if (_mutex_baro)    vSemaphoreDelete(_mutex_baro);
        }

        // 실제 공유 데이터 저장소
        ImuData         _shared_imu_data   {};
        Attitude_t      _currentAttitude   {};
        Attitude_t      _targetAttitude    {}; 
        BaroData        _shared_baro_data  {};
        
        // 데이터 파트별 독립 뮤텍스
        SemaphoreHandle_t _mutex_imu     = nullptr;
        SemaphoreHandle_t _mutex_current = nullptr;
        SemaphoreHandle_t _mutex_target  = nullptr;
        SemaphoreHandle_t _mutex_baro    = nullptr;

        // 💡 [방어 설계] 부팅 후 0점 교정이 완벽히 완료되었는지 멀티코어 간 동기화를 보장하는 전역 플래그
        std::atomic<bool> _is_imu_calibrated;

    public:
        static SharedDataManager& getinstance() {
            static SharedDataManager instance;
            return instance;
        }
        
        SharedDataManager(const SharedDataManager&) = delete;
        SharedDataManager& operator=(const SharedDataManager&) = delete;
        SharedDataManager& operator=(SharedDataManager&&) = delete;

        // --- 1. IMU 데이터 창구 ---
        void update_latest_imu(const ImuData& new_data) {
            if (xSemaphoreTake(_mutex_imu, 0) == pdTRUE) {
                _shared_imu_data = new_data; 
                xSemaphoreGive(_mutex_imu);
            }
        }

        bool get_latest_imu(ImuData& out_data) {
            if (xSemaphoreTake(_mutex_imu, 0) == pdTRUE) {
                out_data = _shared_imu_data;
                xSemaphoreGive(_mutex_imu);
                return true;
            }
            return false; 
        }

        // --- 2. 현재 자세(Attitude) 데이터 창구 ---
        void setAttitude(const Attitude_t& att) {
            if (xSemaphoreTake(_mutex_current, 0) == pdTRUE) {
                _currentAttitude = att;
                xSemaphoreGive(_mutex_current);
            }
        }

        Attitude_t getAttitude() {
            Attitude_t temp {};
            if (xSemaphoreTake(_mutex_current, 0) == pdTRUE) {
                temp = _currentAttitude;
                xSemaphoreGive(_mutex_current);
                return temp;
            }
            return _currentAttitude; 
        }

        // --- 3. 목표 자세(Target Attitude) 데이터 창구 ---
        void setTargetAttitude(const Attitude_t& target) {
            if (xSemaphoreTake(_mutex_target, 0) == pdTRUE) {
                _targetAttitude = target;
                xSemaphoreGive(_mutex_target);
            }
        }

        Attitude_t getTargetAttitude() {
            if (xSemaphoreTake(_mutex_target, 0) == pdTRUE) {
                Attitude_t temp = _targetAttitude;
                xSemaphoreGive(_mutex_target);
                return temp;
            }
            return _targetAttitude; 
        }

        // --- 4. 고도(Baro) 데이터 창구 ---
        void update_latest_baro(const BaroData& new_data) {
            if (xSemaphoreTake(_mutex_baro, 0) == pdTRUE) {
                _shared_baro_data = new_data;
                _shared_baro_data.is_updated = true; // 새 기압 데이터 유입 표시
                xSemaphoreGive(_mutex_baro);
            }
        }

        bool get_latest_baro(BaroData& out_data) {
            if (xSemaphoreTake(_mutex_baro, 0) == pdTRUE) {
                out_data = _shared_baro_data;
                
                // 💡 [방어 설계] 공유 원본의 플래그를 직접 끄는 대신, 읽기 성공 시 호출 측에만 true를 반환하고 
                // 원본 플래그 리셋은 제어 루프의 소비 확인 시점으로 완전히 분리하는 것이 구조적으로 안전합니다.
                _shared_baro_data.is_updated = false; 
                xSemaphoreGive(_mutex_baro);
                return out_data.is_updated; // 가져간 데이터가 최신이었는지 여부 반환
            }
            return false;
        }

        // --- 5. 💡 IMU 캘리브레이션 플래그 창구 추가 ---
        void set_imu_calibrated(bool state) {
            _is_imu_calibrated.store(state, std::memory_order_release);
        }

        bool is_imu_calibrated() {
            return _is_imu_calibrated.load(std::memory_order_acquire);
        }
};

} // namespace Utils
