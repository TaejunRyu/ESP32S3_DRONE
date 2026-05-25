/**
 * @file ryu_SharedDataManager.hpp
 * @version 0.5 (소멸자 및 이동 생성 에러 수정)
 */

#pragma once
#include <atomic> 
#include "ryu_Types.hpp"

namespace Controller {

enum class Data_type {
    DT_IMU_DATA,
    DT_BARO_DATA,
    DT_MAG_DATA,
    DT_CURRENT_ATTITUDE,
    DT_TARGET_ATTITUDE,
    DT_GPS_DATA,
    DT_QGC_ATTITUDE,
    DT_RC_DATA
};

template <Data_type T> struct DataTypeTraits;
template <> struct DataTypeTraits<Data_type::DT_IMU_DATA>           { using Type = SensorData; };
template <> struct DataTypeTraits<Data_type::DT_MAG_DATA>           { using Type = Vector3f; };
template <> struct DataTypeTraits<Data_type::DT_BARO_DATA>          { using Type = BaroData; };
template <> struct DataTypeTraits<Data_type::DT_CURRENT_ATTITUDE>   { using Type = Attitude_t; };
template <> struct DataTypeTraits<Data_type::DT_TARGET_ATTITUDE>    { using Type = Attitude_t; };
template <> struct DataTypeTraits<Data_type::DT_QGC_ATTITUDE>       { using Type = QgcAttitude_t; };
template <> struct DataTypeTraits<Data_type::DT_GPS_DATA>           { using Type = gps_data_t; };
template <> struct DataTypeTraits<Data_type::DT_RC_DATA>            { using Type = rc_data_t; };


class SharedDataManager {
private:
    static constexpr const char* TAG = "SharedDataManager";    
    SharedDataManager(){}
    // [Writers 제약 조건] 
    // 본 더블 버퍼 구조는 각 Data_type별로 데이터를 쓰는 태스크(Writer)가 '단 1개'일 때만 원자성이 보장됩니다.
    SensorData      _imu_buffer[2]       = {};
    BaroData        _baro_buffer[2]      = {};
    Attitude_t      _currentAttitude[2]  = {};
    Attitude_t      _targetAttitude[2]   = {}; 
    QgcAttitude_t   _qgcAttitude[2]      = {}; // MAVLINK로 QGC에 ATTITUDE/SPEED를 위하여 전송 
    gps_data_t      _gps_buffer[2]       = {};    
    Vector3f        _mag_buffer[2]       = {};
    rc_data_t       _rc_buffer[2]        = {};

    std::atomic<int> _imu_latest_idx{0};
    std::atomic<int> _baro_latest_idx{0};
    std::atomic<int> _curatt_latest_idx{0};
    std::atomic<int> _taratt_latest_idx{0};
    std::atomic<int> _qgcatt_latest_idx{0};
    std::atomic<int> _gps_latest_idx{0};
    std::atomic<int> _mag_latest_idx{0};
    std::atomic<int> _rc_latest_idx{0};

    std::atomic<bool> _is_imu_calibrated;
    std::atomic<bool> _is_baro_updated;
    std::atomic<bool> _is_mag_updated;
    std::atomic<bool> _is_gps_updated;
    std::atomic<bool> _is_rc_updated;

    bool _initialized = false;
public:
    esp_err_t initialize(){
        if (_initialized) return ESP_OK;
        _is_imu_calibrated.store(false, std::memory_order_relaxed);
        _is_baro_updated.store(false, std::memory_order_relaxed);
        _is_mag_updated.store(false, std::memory_order_relaxed);        
        _is_gps_updated.store(false, std::memory_order_relaxed);
        _is_rc_updated.store(false, std::memory_order_relaxed);
        _initialized = true;
        return ESP_OK;
    };
    bool is_initialized(){return _initialized;};


    // 중요: static 객체의 정상 소멸을 위해 public으로 이동
    ~SharedDataManager() = default; 

    static SharedDataManager& getInstance() {
        static SharedDataManager instance;
        return instance;
    }       
    
    // 복사 및 이동 전면 제한
    SharedDataManager(const SharedDataManager&) = delete;
    SharedDataManager& operator=(const SharedDataManager&) = delete;
    SharedDataManager(SharedDataManager&&) = delete; // 🛠️ 문법 수정: 복사 차단 확실히 처리
    SharedDataManager& operator=(SharedDataManager&&) = delete;

    // 1. 데이터 게시 (각 데이터 유형별 single-writer 환경 필수)
    template <Data_type TypeEnum>
    void publish_data(const typename DataTypeTraits<TypeEnum>::Type& new_data) {
        if constexpr (TypeEnum == Data_type::DT_IMU_DATA) {
            int write_idx = 1 - _imu_latest_idx.load(std::memory_order_relaxed);
            _imu_buffer[write_idx] = new_data; 
            _imu_latest_idx.store(write_idx, std::memory_order_release);
        } else if constexpr (TypeEnum == Data_type::DT_MAG_DATA) {
            int write_idx = 1 - _mag_latest_idx.load(std::memory_order_relaxed);
            _mag_buffer[write_idx] = new_data; 
            _mag_latest_idx.store(write_idx, std::memory_order_release);        
            _is_mag_updated.store(true, std::memory_order_release); // 💡 게시 시 플래그 연동 자동화
        } else if constexpr (TypeEnum == Data_type::DT_BARO_DATA) {
            int write_idx = 1 - _baro_latest_idx.load(std::memory_order_relaxed);
            _baro_buffer[write_idx] = new_data; 
            _baro_latest_idx.store(write_idx, std::memory_order_release);
            _is_baro_updated.store(true, std::memory_order_release); // 💡 게시 시 플래그 연동 자동화            
        } else if constexpr (TypeEnum == Data_type::DT_CURRENT_ATTITUDE) {
            int write_idx = 1 - _curatt_latest_idx.load(std::memory_order_relaxed);
            _currentAttitude[write_idx] = new_data; 
            _curatt_latest_idx.store(write_idx, std::memory_order_release);
        } else if constexpr (TypeEnum == Data_type::DT_TARGET_ATTITUDE) {
            int write_idx = 1 - _taratt_latest_idx.load(std::memory_order_relaxed);
            _targetAttitude[write_idx] = new_data; 
            _taratt_latest_idx.store(write_idx, std::memory_order_release);
        } else if constexpr (TypeEnum == Data_type::DT_QGC_ATTITUDE) {
            int write_idx = 1 - _qgcatt_latest_idx.load(std::memory_order_relaxed);
            _qgcAttitude[write_idx] = new_data; 
            _qgcatt_latest_idx.store(write_idx, std::memory_order_release);
        } else if constexpr (TypeEnum == Data_type::DT_GPS_DATA) {
            int write_idx = 1 - _gps_latest_idx.load(std::memory_order_relaxed);
            _gps_buffer[write_idx] = new_data; 
            _gps_latest_idx.store(write_idx, std::memory_order_release);
            _is_gps_updated.store(true, std::memory_order_release); // 💡 게시 시 플래그 연동 자동화            
        } else if constexpr (TypeEnum == Data_type::DT_RC_DATA) {
            int write_idx = 1 - _rc_latest_idx.load(std::memory_order_relaxed);
            _rc_buffer[write_idx] = new_data; 
            _rc_latest_idx.store(write_idx, std::memory_order_release);
            _is_rc_updated.store(true, std::memory_order_release); // 💡 게시 시 플래그 연동 자동화            
        }
    }

    // 2. 데이터 획득 (Multi-reader 환경 안전)
    template <Data_type TypeEnum>
    typename DataTypeTraits<TypeEnum>::Type get_shared_data() {
        if constexpr (TypeEnum == Data_type::DT_IMU_DATA) {
            int read_idx = _imu_latest_idx.load(std::memory_order_acquire);
            return _imu_buffer[read_idx]; 
        } else if constexpr (TypeEnum == Data_type::DT_MAG_DATA) {
            int read_idx = _mag_latest_idx.load(std::memory_order_acquire);
            return _mag_buffer[read_idx];
        } else if constexpr (TypeEnum == Data_type::DT_BARO_DATA) {
            int read_idx = _baro_latest_idx.load(std::memory_order_acquire);
            return _baro_buffer[read_idx];
        } else if constexpr (TypeEnum == Data_type::DT_CURRENT_ATTITUDE) {
            int read_idx = _curatt_latest_idx.load(std::memory_order_acquire);
            return _currentAttitude[read_idx]; 
        } else if constexpr (TypeEnum == Data_type::DT_TARGET_ATTITUDE) {
            int read_idx = _taratt_latest_idx.load(std::memory_order_acquire);
            return _targetAttitude[read_idx]; 
        } else if constexpr (TypeEnum == Data_type::DT_QGC_ATTITUDE) {
            int read_idx = _qgcatt_latest_idx.load(std::memory_order_acquire);
            return _qgcAttitude[read_idx]; 
        } else if constexpr (TypeEnum == Data_type::DT_GPS_DATA) {
            int read_idx = _gps_latest_idx.load(std::memory_order_acquire);
            return _gps_buffer[read_idx]; 
        } else if constexpr (TypeEnum == Data_type::DT_RC_DATA) {
            int read_idx = _rc_latest_idx.load(std::memory_order_acquire);
            return _rc_buffer[read_idx]; 
        }
    }

    void set_imu_calibrated(bool state) { _is_imu_calibrated.store(state, std::memory_order_release); }
    bool is_imu_calibrated() { return _is_imu_calibrated.load(std::memory_order_acquire); }


    void set_baro_updated(bool state) { _is_baro_updated.store(state, std::memory_order_release); }
    bool is_baro_updated() { return _is_baro_updated.exchange(false, std::memory_order_acq_rel); } 

    void set_mag_updated(bool state) { _is_mag_updated.store(state, std::memory_order_release); }
    bool is_mag_updated() { return _is_mag_updated.exchange(false, std::memory_order_acq_rel); } 

    void set_gps_updated(bool state) { _is_gps_updated.store(state, std::memory_order_release); }
    bool is_gps_updated() { return _is_gps_updated.exchange(false, std::memory_order_acq_rel); } 

    void set_rc_updated(bool state) { _is_rc_updated.store(state, std::memory_order_release); }
    bool is_rc_updated() { return _is_rc_updated.exchange(false, std::memory_order_acq_rel); } 


    // 2) 일반 관측 태스크 전용(Read-Only Peek): 플래그를 절대 소거하지 않고 순수 업데이트 유무 상태만 단순 조회
    bool peek_baro_updated() { return _is_baro_updated.load(std::memory_order_acquire); }
    bool peek_mag_updated()  { return _is_mag_updated.load(std::memory_order_acquire); }
    bool peek_gps_updated()  { return _is_gps_updated.load(std::memory_order_acquire); }
    bool peek_rc_updated()   { return _is_rc_updated.load(std::memory_order_acquire); }

};

} // namespace Controller
