#pragma once
#include <atomic>
#include "ryu_Types.hpp"

namespace Controller {

// 비행 모드 열거형 정의
enum class flyingMode_e : uint32_t {
    MODE_MANUAL         = 0x00010000u,  
    MODE_ALTCTL         = 0x00020000u,  
    MODE_POSCTL         = 0x00030000u,  
    MODE_OFFBOARD       = 0x00040000u,  
    MODE_ACRO           = 0x00050000u,  
    MODE_STABILIZED     = 0x00070000u,  
    MODE_STANBY         = 0x03040000u,  
    MODE_MISSION        = 0x04040000u,  
    MODE_RTL            = 0x05040000u,  
    MODE_PRECISION_LAND = 0x09040000u,  
    MODE_USER_HOLD_MODE = 0x03040000u,  
};

// 시스템의 현재 상태 (예: 시동 여부, 비행 여부, 센서 캘리브레이션 상태 등)을 나타내는 열거형
enum class systemState_e : uint8_t {                    
    SYS_STATE_UNINIT,            
    SYS_STATE_BOOT,              
    SYS_STATE_CALIBRATING,       
    SYS_STATE_STANDBY,           
    SYS_STATE_ACTIVE,            
    SYS_STATE_CRITICAL,          
    SYS_STATE_EMERGENCY,         
    SYS_STATE_POWEROFF,          
    SYS_STATE_FLIGHT_TERMINATION, 
};


class DroneStatusManager {
private:
    std::atomic<bool>           is_armed{false};
    std::atomic<bool>           is_flying{false};
    std::atomic<bool>           is_imu_calibrated{false};
    std::atomic<bool>           is_baro_calibrated{false};
    std::atomic<flyingMode_e>   flying_mode{flyingMode_e::MODE_STABILIZED};     
    std::atomic<systemState_e>  system_state{systemState_e::SYS_STATE_UNINIT};  

    std::atomic<uint32_t>       update_flags{0};

    DroneStatusManager() = default;

public:
    static constexpr uint32_t FLAG_ARMED           = (1 << 0);
    static constexpr uint32_t FLAG_FLYING          = (1 << 1);
    static constexpr uint32_t FLAG_IMU_CALIB       = (1 << 2);
    static constexpr uint32_t FLAG_BARO_CALIB      = (1 << 3);
    static constexpr uint32_t FLAG_FLYING_MODE     = (1 << 4);
    static constexpr uint32_t FLAG_SYSTEM_STATE    = (1 << 5);

    static DroneStatusManager& getInstance() {
        static DroneStatusManager instance;
        return instance;
    }

    // --------------------------------------------------------------------
    // [💡 PEEK 함수군] 플래그를 리셋하지 않고 단순 상태 조회
    // --------------------------------------------------------------------
    
    // 특정 변수가 업데이트 되었는지 여부만 단순 확인 (플래그 리셋 없음)
    bool peekUpdate(uint32_t flag_mask) const {
        return (update_flags.load() & flag_mask) != 0;
    }

    // 플래그 상관없이 현재 원자적 저장소에 기록된 데이터 값만 즉시 조회 (리셋 없음)
    bool getArmedValue() const { return is_armed.load(); }
    bool getFlyingValue() const { return is_flying.load(); }
    bool getImuCalibratedValue() const { return is_imu_calibrated.load(); }
    bool getBaroCalibratedValue() const { return is_baro_calibrated.load(); }
    flyingMode_e getFlyingModeValue() const { return flying_mode.load(); }
    systemState_e getSystemStateValue() const { return system_state.load(); }


    // --------------------------------------------------------------------
    // [사용자/소비 태스크 전용] 업데이트 확인 및 값 참조 (확인 직후 플래그 자동 리셋)
    // --------------------------------------------------------------------
    
    bool checkAndGetArmed(bool& value) {
        uint32_t expected = update_flags.load();
        while (expected & FLAG_ARMED) {
            uint32_t desired = expected & ~FLAG_ARMED;
            if (update_flags.compare_exchange_weak(expected, desired)) {
                value = is_armed.load(); 
                return true;             
            }
        }
        return false; 
    }

    bool checkAndGetFlying(bool& value) {
        uint32_t expected = update_flags.load();
        while (expected & FLAG_FLYING) {
            uint32_t desired = expected & ~FLAG_FLYING;
            if (update_flags.compare_exchange_weak(expected, desired)) {
                value = is_flying.load();
                return true;
            }
        }
        return false;
    }

    bool checkAndGetImuCalibrated(bool& value) {
        uint32_t expected = update_flags.load();
        while (expected & FLAG_IMU_CALIB) {
            uint32_t desired = expected & ~FLAG_IMU_CALIB;
            if (update_flags.compare_exchange_weak(expected, desired)) {
                value = is_imu_calibrated.load();
                return true;
            }
        }
        return false;
    }

    bool checkAndGetBaroCalibrated(bool& value) {
        uint32_t expected = update_flags.load();
        while (expected & FLAG_BARO_CALIB) {
            uint32_t desired = expected & ~FLAG_BARO_CALIB;
            if (update_flags.compare_exchange_weak(expected, desired)) {
                value = is_baro_calibrated.load();
                return true;
            }
        }
        return false;
    }

    bool checkAndGetFlyingMode(flyingMode_e& value) {
        uint32_t expected = update_flags.load();
        while (expected & FLAG_FLYING_MODE) {
            uint32_t desired = expected & ~FLAG_FLYING_MODE;
            if (update_flags.compare_exchange_weak(expected, desired)) {
                value = flying_mode.load();
                return true;
            }
        }
        return false;
    }

    bool checkAndGetSystemState(systemState_e& value) {
        uint32_t expected = update_flags.load();
        while (expected & FLAG_SYSTEM_STATE) {
            uint32_t desired = expected & ~FLAG_SYSTEM_STATE;
            if (update_flags.compare_exchange_weak(expected, desired)) {
                value = system_state.load();
                return true;
            }
        }
        return false;
    }

    // --------------------------------------------------------------------
    // [산발적 데이터 수정 태스크 전용 Setter] 데이터 반영 및 플래그 업
    // --------------------------------------------------------------------
    void setArmed(bool value) {
        is_armed.store(value);
        update_flags.fetch_or(FLAG_ARMED);
    }

    void setFlying(bool value) {
        is_flying.store(value);
        update_flags.fetch_or(FLAG_FLYING);
    }

    void setImuCalibrated(bool value) {
        is_imu_calibrated.store(value);
        update_flags.fetch_or(FLAG_IMU_CALIB);
    }

    void setBaroCalibrated(bool value) {
        is_baro_calibrated.store(value);
        update_flags.fetch_or(FLAG_BARO_CALIB);
    }

    void setFlyingMode(flyingMode_e value) {
        flying_mode.store(value);
        update_flags.fetch_or(FLAG_FLYING_MODE);
    }

    void setSystemState(systemState_e value) {
        system_state.store(value);
        update_flags.fetch_or(FLAG_SYSTEM_STATE);
    }
};

} // namespace Controller
