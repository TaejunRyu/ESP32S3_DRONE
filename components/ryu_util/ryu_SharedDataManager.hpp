/**
 * @file ryu_SharedDataManager.hpp
 * @brief 
 *      1. Singleton으로 설계하여 필요한 데이터를 바로 가져 갈수 있도록한다.
 *      2. Task Notification 연동 구조 도입으로 뮤텍스를 전면 제거하여 지터(Jitter)를 0으로 만듭니다.
 *      3. 센서 데이터, euler(attitude), 비행 태스크 핸들을 통합 관리합니다. 
 * @version 0.4
 * @date 2026-05-18
 */

#pragma once
#include "ryu_Types.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // 💡 Task Notification 핸들 관리를 위한 헤더 포함
#include <atomic> 

namespace Utils {

enum class Data_type{
    DT_IMU_DATA,
    DT_BARO_DATA,
    DT_MAG_DATA,
    DT_CURRENT_ATTITUDE,
    DT_TARGET_ATTITUDE
};

// 1. Enum과 실제 타입을 매핑하는 구조체 정의
template <Data_type T> struct DataTypeTraits;
template <> struct DataTypeTraits<Data_type::DT_IMU_DATA> { using Type = SensorData; };
template <> struct DataTypeTraits<Data_type::DT_BARO_DATA> { using Type = BaroData; };
template <> struct DataTypeTraits<Data_type::DT_CURRENT_ATTITUDE> { using Type = Attitude_t; };
template <> struct DataTypeTraits<Data_type::DT_TARGET_ATTITUDE> { using Type = Attitude_t; };

class SharedDataManager {
    private:
        static constexpr const char* TAG = "SharedDataManager";    
        SharedDataManager() {
            _is_imu_calibrated.store(false);
            _flight_task_handle.store(nullptr); // 초기화
        }
        ~SharedDataManager() = default; // 뮤텍스가 없으므로 해제 자원도 없음 (메모리 절약)

        // 실제 공유 데이터 저장소 (멀티코어 다이렉트 복사용 캐시)
        SensorData         _shared_imu_data   {};
        Attitude_t      _currentAttitude   {};
        Attitude_t      _targetAttitude    {}; 
        BaroData        _shared_baro_data  {};
        
        // 부팅 후 0점 교정이 완벽히 완료되었는지 동기화를 보장하는 전역 플래그
        std::atomic<bool> _is_imu_calibrated;

        //Core 0와 Core 1을 연결하는 동기화 배턴용 비행 태스크 핸들 원자적 보관소
        std::atomic<TaskHandle_t> _flight_task_handle;

    public:
        static SharedDataManager& getinstance() {
            static SharedDataManager instance;
            return instance;
        }       
        SharedDataManager(const SharedDataManager&) = delete;
        SharedDataManager& operator=(const SharedDataManager&) = delete;
        SharedDataManager& operator=(SharedDataManager&&) = delete;

    

















        // --- 1. IMU 데이터 창구 (뮤텍스 락 완전 제거) ---
        // Core 0 전용: 읽기 태스크와 충돌 가능성이 시각적으로 격리되어 있으므로 락 없이 즉시 갱신
        void update_latest_imu(const SensorData& new_data) {
            _shared_imu_data = new_data; 
        }

        // Core 1 전용: Notification을 받고 들어오므로 무조건 최신 데이터 복사 성공 보장
        bool get_latest_imu(SensorData& out_data) {
            out_data = _shared_imu_data;
            return true; 
        }

        // --- 2. 현재 자세(Attitude) 데이터 창구 ---
        void setAttitude(const Attitude_t& att) {
            _currentAttitude = att;
        }

        Attitude_t getAttitude() {
            return _currentAttitude; 
        }

        // --- 3. 목표 자세(Target Attitude) 데이터 창구 ---
        void setTargetAttitude(const Attitude_t& target) {
            _targetAttitude = target;
        }

        Attitude_t getTargetAttitude() {
            return _targetAttitude; 
        }

        // --- 4. 고도(Baro) 데이터 창구 ---
        void update_latest_baro(const BaroData& new_data) {
            _shared_baro_data = new_data;
            _shared_baro_data.is_updated = true; 
        }

        bool get_latest_baro(BaroData& out_data) {
            out_data = _shared_baro_data;
            _shared_baro_data.is_updated = false; 
            return out_data.is_updated; 
        }


        
        
        
        // --- 5. IMU 캘리브레이션 플래그 창구 ---
        void set_imu_calibrated(bool state) {
            _is_imu_calibrated.store(state, std::memory_order_release);
        }

        bool is_imu_calibrated() {
            return _is_imu_calibrated.load(std::memory_order_acquire);
        }

        // --- 6. 💡 [새로운 중계 기능] 비행 태스크 핸들 중앙 집중 등록 창구 ---
        // main.cpp에서 flight_task를 생성한 후 이 함수로 등록합니다.
        void register_flight_task_handle(TaskHandle_t handle) {
            _flight_task_handle.store(handle, std::memory_order_release);
        }

        // Core 0 (센서 태스크)에서 데이터를 다 채운 후 이 핸들을 꺼내 깨우는 신호를 던집니다.
        TaskHandle_t get_flight_task_handle() {
            return _flight_task_handle.load(std::memory_order_acquire);
        }

};

} // namespace Utils
