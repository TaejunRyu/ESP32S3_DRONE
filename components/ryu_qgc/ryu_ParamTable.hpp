#pragma once
/*
 * 드론의 주요 파라미터를 정의하는 테이블과 관련 함수 선언
 * QGroundControl과의 통신을 위해 MAVLink 프로토콜에 맞춘 데이터 구조로 설계되어 있습니다.
 * 해시 테이블을 사용하여 이름으로 빠르게 파라미터 인덱스를 찾을 수 있도록 구현되어 있습니다.
 * PID 제어 관련 변수와도 연동되어 있어, QGC에서 PID 튜닝이 가능하도록 설계되었습니다.
 */
#include <array>
#include <string_view>
#include <esp_log.h>

namespace Service {

// 3. 파라미터 테이블 정의 (매크로로 간결하게)
#define PARAM_LIST \
    PARAM(BAT1_A_PER_V,15.3910303115845,9)\
    PARAM(BAT1_CAPACITY,-1,9)\
    PARAM(BAT1_CNT_V_CURR,0.000805664050858468,9)\
    PARAM(BAT1_CNT_V_VOLT,0.000805664050858468,9)\
    PARAM(BAT_CRIT_THR,0.0700000002980232,9)\
    PARAM(BAT_EMERGEN_THR,0.0500000007450581,9)\
    PARAM(BAT_LOW_THR,0.150000005960464,9)\
    PARAM(BAT1_N_CELLS,3,6)\
    PARAM(BAT1_R_INTERNAL,-1,9)\
    PARAM(BAT1_SOURCE,0,6)\
    PARAM(BAT1_V_CHARGED,4.05000019073486,9)\
    PARAM(BAT1_V_DIV,10.177939414978,9)\
    PARAM(BAT1_V_EMPTY,3.40000009536743,9)\
    PARAM(BAT1_V_LOAD_DROP,0.300000011920929,9)\
    PARAM(BAT1_V_OFFS_CURR,0,9)\
    PARAM(BAT2_A_PER_V,15.3910303115845,9)\
    PARAM(BAT2_CAPACITY,-1,9)\
    PARAM(BAT2_N_CELLS,3,6)\
    PARAM(BAT2_R_INTERNAL,-1,9)\
    PARAM(BAT2_SOURCE,0,6)\
    PARAM(BAT2_V_CHARGED,4.05000019073486,9)\
    PARAM(BAT2_V_DIV,10.177939414978,9)\
    PARAM(BAT2_V_EMPTY,3.40000009536743,9)\
    PARAM(BAT2_V_LOAD_DROP,0.300000011920929,9)\
    PARAM(CAL_ACC0_ID,1,6)\
    PARAM(CAL_ACC1_ID,0,6)\
    PARAM(CAL_ACC2_ID,0,6)\
    PARAM(CAL_GYRO0_ID,3,6)\
    PARAM(CAL_GYRO1_ID,0,6)\
    PARAM(CAL_GYRO2_ID,0,6)\
    PARAM(CAL_MAG0_ID,1,6)\
    PARAM(CAL_MAG0_ROT,0,6)\
    PARAM(CAL_MAG1_ID,0,6)\
    PARAM(CAL_MAG1_ROT,0,6)\
    PARAM(CAL_MAG2_ID,0,6)\
    PARAM(CAL_MAG2_ROT,0,6)\
    PARAM(COM_DISARM_LAND,2.0f,9)\
    PARAM(COM_DL_LOSS_T,10,6)\
    PARAM(COM_FLTMODE1,-1,6)\
    PARAM(COM_FLTMODE2,-1,6)\
    PARAM(COM_FLTMODE3,-1,6)\
    PARAM(COM_FLTMODE4,-1,6)\
    PARAM(COM_FLTMODE5,-1,6)\
    PARAM(COM_FLTMODE6,-1,6)\
    PARAM(COM_LOW_BAT_ACT,0,6)\
    PARAM(COM_OBS_AVOID,0,6)\
    PARAM(COM_RC_IN_MODE,3,6)\
    PARAM(COM_RC_LOSS_T,0.5f,9)\
    PARAM(CP_DIST,-1.0f,9)\
    PARAM(GF_ACTION,0,6)\
    PARAM(GF_ALTMODE,0,6)\
    PARAM(GF_COUNT,-1,6)\
    PARAM(GF_MAX_HOR_DIST,0	,9)\
    PARAM(GF_MAX_VER_DIST,0,9)\
    PARAM(GF_SOURCE,0,6)\
    PARAM(MAV_SYS_ID,2,6)\
    PARAM(MC_PITCH_P,6.5f,9)\
    PARAM(MC_PITCHRATE_D,0.003f,9)\
    PARAM(MC_PITCHRATE_I,0.1f,9)\
    PARAM(MC_PITCHRATE_K,1.0f,9)\
    PARAM(MC_PITCHRATE_P,0.15,9)\
    PARAM(MC_ROLL_P,6.5f,9)\
    PARAM(MC_ROLLRATE_D,0.003f,9)\
    PARAM(MC_ROLLRATE_I,0.1f,9)\
    PARAM(MC_ROLLRATE_K,1.0f,9)\
    PARAM(MC_ROLLRATE_P,0.15f,9)\
    PARAM(MC_YAW_P,0.3f,9)\
    PARAM(MC_YAWRATE_I,0.04f,9)\
    PARAM(MC_YAWRATE_K,1.0f,9)\
    PARAM(MPC_PITCH_P,4.0f,9)\
    PARAM(MPC_ROLL_P,4.0f,9)\
    PARAM(MPC_XY_P,0.95f,9)\
    PARAM(MPC_XY_VEL_D_ACC,0.01f,9)\
    PARAM(MPC_XY_VEL_I_ACC,0.4f,9)\
    PARAM(MPC_XY_VEL_P_ACC,1.8f,9)\
    PARAM(MPC_YAW_P,2.8f,9)\
    PARAM(MPC_Z_P,1.0f,9)\
    PARAM(MPC_Z_VEL_D_ACC,0.01f,9)\
    PARAM(MPC_Z_VEL_I_ACC,0.4f,9)\
    PARAM(MPC_Z_VEL_P_ACC,1.8f,9)\
    PARAM(MPC_Z_VEL_P,0.2f,9)\
    PARAM(MPC_Z_VEL_I,0.02f,9)\
    PARAM(MPC_Z_VEL_D,0.0f,9)\
    PARAM(NAV_DLL_ACT,0,6)\
    PARAM(NAV_RCL_ACT,2,6)\
    PARAM(PWM_MAIN_MAX,1500.0f,9)\
    PARAM(PWM_MAIN_MIN,1000.0f,9)\
    PARAM(RC_ACRO_TH,0.5f,9)\
    PARAM(RC_ARMSWITCH_TH,0.25f,9)\
    PARAM(RC_ASSIST_TH,0.25f,9)\
    PARAM(RC_AUTO_TH,0.75f,9)\
    PARAM(RC_CHAN_CNT,0,6)\
    PARAM(RC_FAILS_THR,0,6)\
    PARAM(RC_FLT_CUTOFF,10.0f,9)\
    PARAM(RC_FLT_SMP_RATE,50.0f,9)\
    PARAM(RC_GEAR_TH,0.25f,9)\
    PARAM(RC_KILLSWITCH_TH,0.25f,9)\
    PARAM(RC_LOITER_TH,0.5f,9)\
    PARAM(RC_MAN_TH,0.5f,9)\
    PARAM(RC_MAP_ACRO_SW,0,6)\
    PARAM(RC_MAP_ARM_SW,0,6)\
    PARAM(RC_MAP_AUX1,0,6)\
    PARAM(RC_MAP_AUX2,0,6)\
    PARAM(RC_MAP_AUX3,0,6)\
    PARAM(RC_MAP_AUX4,0,6)\
    PARAM(RC_MAP_AUX5,0,6)\
    PARAM(RC_MAP_FAILSAFE,0,6)\
    PARAM(RC_MAP_FLAPS,0,6)\
    PARAM(RC_MAP_FLTMODE,0,6)\
    PARAM(RC_MAP_GEAR_SW,0,6)\
    PARAM(RC_MAP_KILL_SW,0,6)\
    PARAM(RC_MAP_LOITER_SW,0,6)\
    PARAM(RC_MAP_MAN_SW,0,6)\
    PARAM(RC_MAP_MODE_SW,0,6)\
    PARAM(RC_MAP_OFFB_SW,0,6)\
    PARAM(RC_MAP_PARAM1,0,6)\
    PARAM(RC_MAP_PARAM2,0,6)\
    PARAM(RC_MAP_PARAM3,0,6)\
    PARAM(RC_MAP_PITCH,2,6)\
    PARAM(RC_MAP_POSCTL_SW,0,6)\
    PARAM(RC_MAP_RATT_SW,0,6)\
    PARAM(RC_MAP_RETURN_SW,0,6)\
    PARAM(RC_MAP_ROLL,1,6)\
    PARAM(RC_MAP_STAB_SW,0,6)\
    PARAM(RC_MAP_THROTTLE,3,6)\
    PARAM(RC_MAP_TRANS_SW,0,6)\
    PARAM(RC_MAP_YAW,4,6)\
    PARAM(RC_OFFB_TH,0.5f,9)\
    PARAM(RC_POSCTL_TH,0.5f,9)\
    PARAM(RC_RATT_TH,0.5f,9)\
    PARAM(RC_RETURN_TH,0.5f,9)\
    PARAM(RC_STAB_TH,0.5f,9)\
    PARAM(RC_TRANS_TH,0.25f,9)\
    PARAM(RC1_DZ,10,9)\
    PARAM(RC1_MAX,2000,9)\
    PARAM(RC1_MIN,1000,9)\
    PARAM(RC1_REV,1,9)\
    PARAM(RC1_TRIM,1500,9)\
    PARAM(RC10_DZ,0,9)\
    PARAM(RC10_MAX,2000,9)\
    PARAM(RC10_MIN,1000,9)\
    PARAM(RC10_REV,1,9)\
    PARAM(RC10_TRIM,1500,9)\
    PARAM(RC11_DZ,0,9)\
    PARAM(RC11_MAX,2000,9)\
    PARAM(RC11_MIN,1000,9)\
    PARAM(RC11_REV,1,9)\
    PARAM(RC11_TRIM,1500,9)\
    PARAM(RC12_DZ,0,9)\
    PARAM(RC12_MAX,2000,9)\
    PARAM(RC12_MIN,1000,9)\
    PARAM(RC12_REV,1,9)\
    PARAM(RC12_TRIM,1500,9)\
    PARAM(RC13_DZ,0,9)\
    PARAM(RC13_MAX,2000,9)\
    PARAM(RC13_MIN,1000,9)\
    PARAM(RC13_REV,1,9)\
    PARAM(RC13_TRIM,1500,9)\
    PARAM(RC14_DZ,0,9)\
    PARAM(RC14_MAX,2000,9)\
    PARAM(RC14_MIN,1000,9)\
    PARAM(RC14_REV,1,9)\
    PARAM(RC14_TRIM,1500,9)\
    PARAM(RC15_DZ,0,9)\
    PARAM(RC15_MAX,2000,9)\
    PARAM(RC15_MIN,1000,9)\
    PARAM(RC15_REV,1,9)\
    PARAM(RC15_TRIM,1500,9)\
    PARAM(RC16_DZ,0,9)\
    PARAM(RC16_MAX,2000,9)\
    PARAM(RC16_MIN,1000,9)\
    PARAM(RC16_REV,1,9)\
    PARAM(RC16_TRIM,1500,9)\
    PARAM(RC17_DZ,0,9)\
    PARAM(RC17_MAX,2000,9)\
    PARAM(RC17_MIN,1000,9)\
    PARAM(RC17_REV,1,9)\
    PARAM(RC17_TRIM,1500,9)\
    PARAM(RC18_DZ,0,9)\
    PARAM(RC18_MAX,2000,9)\
    PARAM(RC18_MIN,1000,9)\
    PARAM(RC18_REV,1,9)\
    PARAM(RC18_TRIM,1500,9)\
    PARAM(RC2_DZ,10,9)\
    PARAM(RC2_MAX,2000,9)\
    PARAM(RC2_MIN,1000,9)\
    PARAM(RC2_REV,1,9)\
    PARAM(RC2_TRIM,1500,9)\
    PARAM(RC3_DZ,10,9)\
    PARAM(RC3_MAX,2000,9)\
    PARAM(RC3_MIN,1000,9)\
    PARAM(RC3_REV,1,9)\
    PARAM(RC3_TRIM,1500,9)\
    PARAM(RC4_DZ,10,9)\
    PARAM(RC4_MAX,2000,9)\
    PARAM(RC4_MIN,1000,9)\
    PARAM(RC4_REV,1,9)\
    PARAM(RC4_TRIM,1500,9)\
    PARAM(RC5_DZ,10,9)\
    PARAM(RC5_MAX,2000,9)\
    PARAM(RC5_MIN,1000,9)\
    PARAM(RC5_REV,1,9)\
    PARAM(RC5_TRIM,1500,9)\
    PARAM(RC6_DZ,10,9)\
    PARAM(RC6_MAX,2000,9)\
    PARAM(RC6_MIN,1000,9)\
    PARAM(RC6_REV,1,9)\
    PARAM(RC6_TRIM,1500,9)\
    PARAM(RC7_DZ,10,9)\
    PARAM(RC7_MAX,2000,9)\
    PARAM(RC7_MIN,1000,9)\
    PARAM(RC7_REV,1,9)\
    PARAM(RC7_TRIM,1500,9)\
    PARAM(RC8_DZ,10,9)\
    PARAM(RC8_MAX,2000,9)\
    PARAM(RC8_MIN,1000,9)\
    PARAM(RC8_REV,1,9)\
    PARAM(RC8_TRIM,1500,9)\
    PARAM(RC9_DZ,0,9)\
    PARAM(RC9_MAX,2000,9)\
    PARAM(RC9_MIN,1000,9)\
    PARAM(RC9_REV,1,9)\
    PARAM(RC9_TRIM,1500,9)\
    PARAM(RTL_DESCEND_ALT,30.0f,9)\
    PARAM(RTL_LAND_DELAY,0.0F,9)\
    PARAM(RTL_RETURN_ALT,60.0f,9)\
    PARAM(SDLOG_MODE,0,6)\
    PARAM(SENS_BOARD_ROT,0,6)\
    PARAM(SENS_DPRES_OFF,0.0f,9)\
    PARAM(SYS_AUTOCONFIG,0,6)\
    PARAM(SYS_AUTOSTART,4001,6)\
    PARAM(SYS_HAS_MAG,1,6)

    struct ParamInfo {
        std::string_view name;
        uint8_t type;
    };

    #define PARAM(name, default_val, type) ParamInfo{ #name, type },
    // const를 붙여 Flash(ROData) 영역에 할당
    inline const struct ParamInfo params[] = { PARAM_LIST };
    #undef PARAM

    class ParamMgr {
    public:
        // 1. 데이터 구조체 정의 (모든 멤버가 float임이 보장됨)
        #define PARAM(name, default_val, type) float name;
        struct ParamValuesStruct {
            PARAM_LIST
        };
        #undef PARAM

        // 싱글톤 접근
        static ParamMgr& get_instance() {
            static ParamMgr instance;
            return instance;
        }

        // 1. 값 읽기 전용 참조 (복사 방지 및 안전성)
        const ParamValuesStruct& get_values() const { return _values; }

        // 2. 이름으로 파라미터 찾아서 수정
        bool update_by_name(std::string_view name, float value) {
            int idx = find_name_index(name);
            if (idx == -1) return false;
            update_by_index(idx, value);
            return true;
        }
        
        // 3. 인덱스로 직접 수정 (최적화된 경로)
        void update_by_index(size_t index, float value) {
            if (index >= param_count) return;
            // 모든 멤버가 float이므로 포인터 배열처럼 접근 가능
            float* ptr = reinterpret_cast<float*>(&_values);
            ptr[index] = value;
        }

        float get_value_by_index(size_t index) const {
            if (index >= param_count) return 0.0f; // 범위 체크
            const float* ptr = reinterpret_cast<const float*>(&_values);
            return ptr[index];
        }
        
        int find_name_index(char *name) const {
            for (size_t i = 0; i < param_count; ++i) {                
                if (params[i].name == std::string_view(name)) return (int)i;
            }
            return -1;
        }

        // 이름 기반 순차 검색
        int find_name_index(std::string_view name) const {
            for (size_t i = 0; i < param_count; ++i) {
                if (params[i].name == name) return (int)i;
            }
            return -1;
        }        


        size_t get_param_count(){return param_count;}

    private:
        // 생성자: INITIAL_VALUES를 사용하여 _values 초기화
        ParamMgr() : _values(INITIAL_VALUES) {}

        ParamValuesStruct _values;
        static constexpr size_t param_count = sizeof(ParamValuesStruct) / sizeof(float);

        // 초기값 리스트 생성을 위한 매크로 정의
        #define PARAM(name, def, type) static_cast<float>(def),
        static inline constexpr ParamValuesStruct INITIAL_VALUES = { PARAM_LIST };
        #undef PARAM        
    };
} //namespace PARAM
